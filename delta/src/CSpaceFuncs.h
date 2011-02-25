#include "command_publisher.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include<tf/transform_listener.h>
#include "opencv2/core/core.hpp"
#include <list>
#include <math.h>
#include <algorithm>
#include <iostream>

#ifndef CSPACEFUNCS_h
#define CSPACEFUNCS_h
using namespace cv;
Mat_<bool>* getMap(const nav_msgs::OccupancyGrid& grid);
void PlotMap(list<geometry_msgs::Point> PointsToPlot, ros::NodeHandle *vis_pub, float r, float g, float b, float cubesize);
#endif
