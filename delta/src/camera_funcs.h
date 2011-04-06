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

using namespace cv;
using namespace std;

void getOrangeLines(Mat& img, vector<Vec4i>& lines);
void popNearest(list<Point2d>& Points, list<Point2d>& OrderedPoints, Point2d Target, double minSeparation);
list<Point2i> getUnsortedPoints(vector<Vec4i> lines);
list<Point2d> linesToNastyPolyLine(list<Point2d> Lines,  double IMAGE_ORIGIN_X, double IMAGE_ORIGIN_Y, double minSeparation);
list<Point2d> cleanNastyPolyLine(list<Point2d> NastyPolyLine, int NumRemaining =20);
list<Point2d> noTransform(list<Point2i> Pts);