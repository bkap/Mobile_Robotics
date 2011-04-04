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

void getOrangeLines(Mat& img, vector<Vec4i>& lines);
void GetNearest(&list<Point2i> Points, &list<Point2i> OrderedPoints, Point2i Target);
list<Point2i> linesToNastyPolyLine(vector<Vec4i> lines);
list<Point2i> cleanNastyPolyLine(list<Point2i> NastyPolyLine, int NumRemaining);