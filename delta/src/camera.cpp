//#include <camera_info_manager/camera_info_manager.h>
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

#define CAMERA_ROI_FILE "cameraROI_base_link"

using namespace cv;
using namespace std;

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
	Mat_<T> temp = Mat(2,sizes,type,data);
	temp.copyTo(mat);
	delete[] data;
}




Mat cameraMat; //intrinsic parameters
Mat distMat; //distortion parameters
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
void findOrange(cv::Mat& src)
{
  Mat temp;
  temp = src;

  //Make a vector of Mats to hold the invidiual B,G,R channels
  vector<Mat> mats;
  //Split the input into 3 separate channels
  split(temp, mats);
  // Set all values below value to zero, leave rest the same
  // Then inverse binary threshold the remaining pixels
  // Threshold blue channel
  threshold(mats[0], mats[0], 60, 255, THRESH_TOZERO_INV);
  threshold(mats[0], mats[0], 0, 255, THRESH_BINARY);
  // Threshold green channel
  threshold(mats[1], mats[1], 120, 255, THRESH_TOZERO_INV);
  threshold(mats[1], mats[1], 0, 255, THRESH_BINARY);
  // Threshold red channel
  threshold(mats[2], mats[2], 255, 255, THRESH_TOZERO_INV);
  threshold(mats[2], mats[2], 180, 255, THRESH_BINARY);
  multiply(mats[0], mats[1], src);
  multiply(src, mats[2], src);
  erode(src, src, Mat());
  dilate(src, src, Mat(), Point(-1,-1), 5);
}
/*
	converts vector<Point2f> of camera coordinates to corresponding PointCloud in map coordinates
*/
void DemoNode::transformPoints(vector<Point2f>& viewPoints, sensor_msgs::PointCloud& mapCloud){

	sensor_msgs::PointCloud cloud;
	cloud.header.frame_id = "base_laser1_link";
	/* transform to base_link */
	Mat_<Point2f> basePoints_;
	perspectiveTransform(Mat(viewPoints),basePoints_,viewToBase);
	
	vector<Point2f> points;
	for(unsigned int i = 0;i<viewPoints.size();i++){
		points.push_back(basePoints_(i));
	}
	
	/*convert to point cloud*/
	//cloud.points.erase(cloud.points.begin(), cloud.points.end());
	for(unsigned int i=0;i<points.size();i++){
		geometry_msgs::Point32 geoPoint;
		geoPoint.x =  points[i].x;
		geoPoint.y =  points[i].y;
		geoPoint.z = 0;
		cloud.points.push_back(geoPoint);
	}

	/*transform cloud to map*/
	tfl->transformPointCloud("map", cloud, mapCloud);
}

void normalizeColor(cv::Mat& img){
	cv::MatIterator_<cv::Vec<uchar,3> > it=img.begin<cv::Vec<uchar,3> >(),it_end=img.end<cv::Vec<uchar,3> >();
	cv::Vec<uchar,3> p;
	for(;it!=it_end;++it){
		//p=*it; 
		double scale = (*it)[0]+(*it)[1]+(*it)[2];
		//p = scale * *it;// (255.0/scale));
		*it = cv::Vec<uchar,3> (cv::saturate_cast<uchar> (255.0 / scale* (float)(*it)[0]),cv::saturate_cast<uchar>(255.0 / scale * (float)(*it)[1]),cv::saturate_cast<uchar>(255.0 /scale * (float)(*it)[2]));
	}
}


void findPoints(Mat& image, vector<Point2f>& points){
	normalizeColor(image);
	//cvNamedWindow("image");
	//imshow("image",image);
	//waitKey(2);
	findOrange(image);
	//cvNamedWindow("orange");
	//imshow("orange",image);
	//waitKey(2);
	vector<vector<Point> > points_;
	findContours(image, points_, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	//Mat outline(image.size(),(uchar)0);
	//drawContours(outline,points_,-1,255);
	//cvNamedWindow("outline");
	//imshow("outline",outline);
	//waitKey(2);
	for(int i =0;i<points_.size();i++){
		for(int j=0;j<points_[i].size();j++){
			points.push_back(Point2f(points_[i][j].x, points_[i][j].y));
		}
	}
}

void DemoNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  //ROS_INFO("image callback");

  static sensor_msgs::CvBridge bridge;
  static bool init = false;
  static vector<Point2f> viewCorners;
  cv::Mat image;
  // Convert image from ROS format to OpenCV format
  try
  {
	image = cv::Mat(bridge.imgMsgToCv(msg, "bgr8"));
	if(!init){
		viewCorners.push_back(Point2f(0,0));
		viewCorners.push_back(Point2f(0,image.size().width-1));
		viewCorners.push_back(Point2f(image.size().height-1,image.size().width-1));
		viewCorners.push_back(Point2f(image.size().height-1,0));
	}
	
	vector<Point2f> points;
	findPoints(image,points);
	transformPoints(viewCorners,viewCloud);
	pub_view_pts.publish(viewCloud);
	if(points.size() > 1){
		transformPoints(points,pointCloud);
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


DemoNode::DemoNode():
  it_(nh_)
{
        readMat(viewToBase,"viewToBase");
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
	DemoNode motion_tracker;

	ROS_INFO("Camera Node Started");
	while(ros::ok()){ros::spinOnce();}
	return 0;
}
