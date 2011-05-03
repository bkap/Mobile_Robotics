#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <eecs376_vision/lib_demo.h>
#include<tf/transform_listener.h> 
#include <stdio.h>
#include <list>
#include "camera_funcs.h"
#include "cvFuncs.h"


using namespace cv;
using namespace std;

/*	Connor Balin
	
	The camera node identifies contours of orange things and tells the mapper about them.  It also tells the mapper where it can see.
	Orange pixels are identified by thresholding the L1 normalized color.  Noise is reduced by killing small blobs and holes with sequential erosion and dilation
	Boundary pixels of remaining blobs are determined by OpenCV's findContours.
	The co-ordinates of these pixels are transformed into map co-ordinates and shoved into a PointCloud to be published.

	The change from centroids to contours was made for because contours are better.
	
	First, centroids are a poor measure of the location or extents of large
	or orange-ratchet-strap-like objects.  The mapper would have no reliable way of guaranteeing sufficient fattening based only on centroids.
	
	Secondly, contours are quite likely to have at least some points which lie on the ground, whereas the centroid pretty
	much never lies on the ground for a sizable obstacle. This again increases the localization accuracy of the contour.
	Since our mapper masks non-ground-plane camera points to the best of its ability, centroids of tall things would be thrown away.  While
	for any single obstacle, this would not be a problem since the lidar picks it up, configurations involving abutting barrels and ribbons
	could be constructed which would falsely fail to detect the ribbon if the centroid were thrown away and falsely project the barrel onto
	the ground if it were not.
	
*/

// Function to load a matrix from a file
template <typename T>
void readMat(cv::Mat_<T>& mat, char* file){
	ifstream* infile = new ifstream(file,ifstream::in|ifstream::binary);
	int rows = 0,cols = 0,type=0,size=0;
	(*infile)>>rows;
	(*infile)>>cols;
	(*infile)>>type;
	(*infile)>>size;
	char* data = new char[size];
	infile->read(data,size);
	infile->close();

	int sizes[2] = {rows,cols};
	Mat_<T> temp = Mat(rows, cols,type,data);
	temp.copyTo(mat);
	delete[] data;
}

tf::TransformListener *tfl;

class DemoNode {
	public:
		DemoNode();
		void publishNavLoc(list<Point2f> NavPoints);
		void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
		list<Point2f> transformPts(list<Point2i> NavPoints);
		ros::NodeHandle nh_; // Made this public to access it
	private:
		image_transport::ImageTransport it_;
		image_transport::Subscriber sub_image_;
		ros::Publisher pub_nav_pts,pub_view_pts;
		sensor_msgs::PointCloud viewCloud;
		sensor_msgs::PointCloud pointCloud;
		void transformPoints(vector<Point2f>& viewPoints, sensor_msgs::PointCloud& mapCloud);
		Mat_<double> viewToBase;
};

/*
	binarizes an image by settng all non-orange pixels to zero and all orange pixels to 255
*/
void findOrange(cv::Mat& src,cv::Mat& dst)
{
	static const Scalar_<uchar> lower(0,0,180),
				    upper(60,120,255);
	inRange(src,lower,upper,dst);

	// Erode and dilate to get rid of noise
	erode(dst, dst, Mat());
	dilate(dst, dst, Mat(), Point(-1,-1), 7);
}

/*
	shoves a vector<Point2f> of camera coordinates into a PointCloud and transforms to map coordinates
*/
void DemoNode::transformPoints(vector<Point2f>& viewPoints, sensor_msgs::PointCloud& mapCloud){

	/* transform to base_link */
	Mat_<Point2f> basePoints_;
	perspectiveTransform(Mat(viewPoints),basePoints_,viewToBase);
	
	/*convert to ROS point cloud type*/
	sensor_msgs::PointCloud cloud;
	cloud.header.frame_id = "base_laser1_link";
	for(unsigned int i = 0;i<viewPoints.size();i++){
		geometry_msgs::Point32 point;
		point.x = basePoints_(i).x;
		point.y = basePoints_(i).y;
		cloud.points.push_back(point);
	}
	
	/*transform cloud to map coordinates*/
	tfl->transformPointCloud("map", cloud, mapCloud);
}

/*
	in-place L1 color normalization of a 3 channel Mat_<uchar>
	not typesafe because typesafety is no fun
*/
void normalizeColor(cv::Mat& img){
	cv::MatIterator_<cv::Vec<uchar,3> > it=img.begin<cv::Vec<uchar,3> >(),it_end=img.end<cv::Vec<uchar,3> >();
	cv::Vec<uchar,3> p;
	for(;it!=it_end;++it){
		// To save processing time, the scale is R+G+B instead of Sqrt(R+G+B)
		double scale = (*it)[0]+(*it)[1]+(*it)[2];
		*it = cv::Vec<uchar,3> (cv::saturate_cast<uchar> (255.0 / scale* (float)(*it)[0]),cv::saturate_cast<uchar>(255.0 / scale * (float)(*it)[1]),cv::saturate_cast<uchar>(255.0 /scale * (float)(*it)[2]));
	}
}

/*
	populates the input vector with points containing pixel coordinates
	of boundaries of orange blobs
*/
void findPoints(Mat& image, vector<Point2f>& points){
	// Normalize colors and find orange pixels
	normalizeColor(image);
	Mat orange = Mat::zeros(image.rows,image.cols,CV_8U);
	findOrange(image,orange);
	
	// Reduce the number of points by finding contours in the image
	vector<vector<Point> > points_;
	findContours(orange, points_, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	// Package the points in a vector
	int s = 0;
	for(int i =0;i<points_.size();i++){
		s+=points_[i].size();
		for(int j=0;j<points_[i].size();j++){
			points.push_back(Point2f(points_[i][j].y, points_[i][j].x));
		}
	}
	ROS_INFO("CAMERA FINDS %d points",s);
}

// This is called whenever the node gets a new image from the camera
void DemoNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Convert image from ROS format to OpenCV format
  static sensor_msgs::CvBridge bridge;
  static bool init = false;
  static vector<Point2f> viewCorners,foundPoints;
  cv::Mat image;
  try
  {
	image = cv::Mat(bridge.imgMsgToCv(msg, "bgr8"));
	if(!init){
		// Store the corners of the image for transformation purposes
		viewCorners.push_back(Point2f(0,0));
		viewCorners.push_back(Point2f(0,image.size().width-1));
		viewCorners.push_back(Point2f(image.size().height-1,image.size().width-1));
		viewCorners.push_back(Point2f(image.size().height-1,0));
	}
	
	transformPoints(viewCorners,viewCloud);
	pub_view_pts.publish(viewCloud);
	
	// Detect orange points in the image
	foundPoints.clear();
	findPoints(image,foundPoints);
	if(foundPoints.size() > 1){
		// Transform and publish points if you got any
		transformPoints(foundPoints,pointCloud);
		pub_nav_pts.publish(pointCloud);
	}
	else{
		//ROS_INFO("Camera: nothing detected");
	}
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'. E was %s", msg->encoding.c_str(), e.what());
  }

}

// Called on node initialization
DemoNode::DemoNode():
  it_(nh_)
{
	// Read in camera calibration parameters
	readMat(viewToBase,"viewToBase");
	// Subscribe to the appropriate image
	sub_image_ = it_.subscribe("image", 1, &DemoNode::imageCallback, this);

  // Publish a cloud of points on the orange line
	pub_nav_pts = nh_.advertise<sensor_msgs::PointCloud>("Camera_Cloud", 1);
	pub_view_pts =nh_.advertise<sensor_msgs::PointCloud>("Camera_view",1);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CameraNode");
	cout << "Camera Initialized" << endl;

	// Wait until you get transform data
	tfl = new tf::TransformListener();
	while (ros::ok()&&!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce();
	cout << "READY"  << endl;
	
	// Initialize the node
	DemoNode motion_tracker;
	
	// Just spin forever, all of the action happens in the camera callback
	ROS_INFO("Camera Node Started");
	ros::spin();
	
	return 0;
}
