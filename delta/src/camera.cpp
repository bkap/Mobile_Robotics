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

using namespace cv;
using namespace std;
void ReadMat(Mat_<float> *mat, char* file);

Mat cameraMat; //intrinsic parameters
Mat distMat; //distortion parameters
tf::TransformListener *tfl;

class DemoNode {
	public:
		DemoNode();
		void publishNavLoc(list<Point2d> NavPoints);
		void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
		void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
		list<Point2d> transformPts(list<Point2i> NavPoints);
		ros::NodeHandle nh_; // Made this public to access it
	private:
		image_transport::ImageTransport it_;
		image_transport::Subscriber sub_image_;
		ros::Subscriber sub_info_;
		ros::Publisher pub_nav_pts;
		sensor_msgs::PointCloud NavPts;
		Mat_<float> rvec, tvec;
		Mat_<double> rvec_, tvec_;
		Mat R2;
		Mat_<double> projector;
};

list<Point2d> DemoNode::transformPts(list<Point2i> NavPoints)
{
	list<Point2d> result; 
	Point2d temp;
	Point3d V;
	
	for(list<Point2i>::iterator it = NavPoints.begin(); it!=NavPoints.end(); it++)
	{
		V.x = it->x;
		V.y = it->y;
		V.z = 0;
		
		Mat_<double> W = projector*Mat(V);
		temp.x = W(0);
		temp.y = W(1);

		cout<<"CAM:Nav Point at "<<temp.x<<","<<temp.y<<" with respect to the robot\n";
		result.push_back(temp);
	}
	
	return result;
}

//call this with a point32 to publish the blob's location
void DemoNode::publishNavLoc(list<Point2d> NavPoints)
{
	NavPts.points.erase(NavPts.points.begin(), NavPts.points.end());
  // Convert into geometry points, and transform from pixel to robot coordinates
	while(NavPoints.size()>0)
	{
		geometry_msgs::Point32 geoPoint;
		geoPoint.x = NavPoints.begin()->x;
		geoPoint.y = NavPoints.begin()->y;
		geoPoint.z = 0;
		NavPts.points.push_back(geoPoint);
		NavPoints.pop_front();
	}
	

  // Transform the point cloud from robot coordinates to map coordinates
	sensor_msgs::PointCloud tNavPts;
	tfl->transformPointCloud("map", NavPts, tNavPts);
	pub_nav_pts.publish(tNavPts);
}

DemoNode::DemoNode():
  it_(nh_)
{
  // Read the extrinsic calibration parameters
	cout << "reading mats" << endl;
	ReadMat(&rvec, "/home/jinx/ROSCode/delta/Mobile_Robotics/rvec");
	ReadMat(&tvec, "/home/jinx/ROSCode/delta/Mobile_Robotics/tvec");
	cout << "read mats" << endl;

  // Subscribe to the image and camera info
	sub_image_ = it_.subscribe("image", 1, &DemoNode::imageCallback, this);
	sub_info_  = nh_.subscribe<sensor_msgs::CameraInfo>("camera_info",1,&DemoNode::infoCallback,this);

  // Publish a cloud of points on the orange line
	pub_nav_pts = nh_.advertise<sensor_msgs::PointCloud>("Cam_Cloud", 1);
 
	NavPts.header.frame_id = "base_laser1_link";
	
	//rvec_ = Mat(&rvec);
	//tvec_ = Mat(&tvec);
	cout<<"Pre-Rodrigues\n";
	Rodrigues(rvec, R2);
	cout<<"Post-Rodrigues\n";
	R2.col(1) = R2.col(2);
	R2.col(2) = tvec;

}

bool cameraCalled = false;
// Callback for CameraInfo (intrinsic parameters)
void DemoNode::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
	const double* K = (msg->K).data();
	const double* D = (msg->D).data();
	Mat(3,3,CV_64F,const_cast<double*>(K)).assignTo(cameraMat,CV_32F);
	Mat(5,1,CV_64F,const_cast<double*>(D)).assignTo(distMat,CV_32F);
	cameraCalled = true;
	//cout<<"I GOT CAMERA INFO!!!!!!!!!!!!\n";
	cout<<"Projector\n";
	projector = Mat_<double>((Mat_<double>(cameraMat) *Mat_<double>( R2)).inv());
}

// Called whenever you get a new image
void DemoNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Don't do anything until you get the intrinsic parameters
  if(!cameraCalled) 
  {
    return;
  }

  cout << "imageCallback" << endl;

  sensor_msgs::CvBridge bridge;
  cv::Mat image;
  // Convert image from ROS format to OpenCV format
  try
  {
    image = cv::Mat(bridge.imgMsgToCv(msg, "bgr8"));
    vector<Vec4i> vickyTheVector;
    getOrangeLines(image, vickyTheVector);
    list<Point2i> imPts = getUnsortedPoints(vickyTheVector);
    list<Point2d> crtPts = transformPts(imPts); 
    crtPts = linesToNastyPolyLine(crtPts, 0, 0, .3);
    crtPts = cleanNastyPolyLine(crtPts, 5);
    if(vickyTheVector.size()==0)
    {
	this->publishNavLoc(crtPts);
    }
    else
    {
	ROS_INFO("CAM: LOLWUT< I DIDN'T SEE ANY LINES!!!!!!\n");
    }
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'. E was %s", msg->encoding.c_str(), e.what());
  }
}

//reads a mat from the file in ~/.ros
void ReadMat(Mat_<float> *mat, char* file)
{
  //cout<<"camout1\n";
  ifstream* infile = new ifstream(file, ifstream::in&ifstream::binary);
  cout << "file opened" << endl;
  int rows, cols, type;
  rows = cols = type = 0;
  (*infile)>>rows;
  cout<<"Scanned Rows, there are/is "<<rows<<" of them\n";
  (*infile)>>cols;
  cout<<"Scanned Columns, there are/is "<<cols<<" of them\n";
  (*infile)>>type;
  cout<<"Scanned Type that number is like "<<type<<" kthxbai\n";
  //cout<<"camout2\n";
  *mat = Mat_<float> (rows, cols); 
  //cout<<"camout3\n";
  unsigned int i, j;
  float f;
	for (i = 0; i < mat->rows; i++)
	{
		//fprintf(mfile,"\n");
		for (j = 0; j < mat->cols; j++)
		{
      (*infile)>>f;
			((*mat)(i,j)) = f;
			cout << f << ",";
		}     
		cout << endl;
	}
	//cout<<"camout4\n";
	infile->close();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "CameraNode");
	cout << "Camera Initialized" << endl;

  // Wait until you get transform data
	tfl = new tf::TransformListener();
	while (!ros::ok()&&!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce();

	DemoNode motion_tracker;

	ROS_INFO("Camera Node Started");
	while(ros::ok()){ros::spinOnce();}
	return 0;
}
