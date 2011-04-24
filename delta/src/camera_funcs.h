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

#ifndef __ORANGELINES_H
#define __ORANGELINES_H
using namespace cv;
using namespace std;

Mat getOrangeLines(Mat& img, vector<Vec4i>& lines);
void popNearest(list<Point2f>& Points, list<Point2f>& OrderedPoints, Point2f Target, double minSeparation);
list<Point2i> getUnsortedPoints(vector<Vec4i> lines);
list<Point2f> linesToNastyPolyLine(list<Point2f> Lines,  double IMAGE_ORIGIN_X, double IMAGE_ORIGIN_Y, double minSeparation);
list<Point2f> cleanNastyPolyLine(list<Point2f> NastyPolyLine, int NumRemaining =20);
list<Point2f> noTransform(list<Point2i> Pts);
#endif
