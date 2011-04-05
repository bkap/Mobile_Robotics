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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CameraNode");
	cout << "Camera Initialized" << endl;
	cout << "READY"  << endl;
	ROS_INFO("Camera Node Started");
 
  cvNamedWindow("detected lines");
  cvNamedWindow("original");
  Mat img = imread("/home/bk/code/dev_stacks/Mobile_Robotics/delta/frame.jpg", 1);
  imshow("original",img);
  Mat img2 = Mat(img);
  vector<Vec4i> lines;
  getOrangeLines(img,lines);
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }

  cvNamedWindow("cleanLine");
  list<Point2i> PL = linesToNastyPolyLine(lines);
  PL = cleanNastyPolyLine(PL);
  
  for(list<Point2i>::iterator it = PL.begin(); it!=PL.end(); it++)
  {
	  line(img2, *it, *(++it--), Scalar(0,0,255),3, CV_AA);
  }
  
  imshow("cleanLine", img);
  imshow("detected lines",img);
  waitKey(-1);
  cvDestroyWindow("detected lines");
  cvDestroyWindow("original");
  cvDestroyWindow("thresholded image");

	while(ros::ok()){ros::spinOnce();}
	return 0;
}
