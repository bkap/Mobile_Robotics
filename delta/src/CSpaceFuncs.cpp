#include "command_publisher.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include<tf/transform_listener.h>
#include "opencv2/core/core.hpp"
#include <list>
#include <math.h>
#include <algorithm>
#include <iostream>

#ifndef CSPACEFUNCS_cpp
#define CSPACEFUNCS_cpp

using namespace cv;
/**assume orientation and resolution are the same*/
Mat_<bool>* getMap(const nav_msgs::OccupancyGrid& grid) {
	Mat_<bool>* m = new Mat_<bool>(grid.info.width, grid.info.height);
	for(unsigned int i = 0; i < grid.info.height; i ++) {
		for(unsigned int j = 0; j < grid.info.width; j++) {
			(*m)(i,j) = (grid.data[i * grid.info.width + j] > 10);
		}
	}
	return m;

}

#endif