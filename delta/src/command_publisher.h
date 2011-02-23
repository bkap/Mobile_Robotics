#include <ros/ros.h>
#include "opencv2/core/core.hpp"
#include <nav_msgs/OccupancyGrid.h>
#ifndef _COMMAND_PUBLISHER_H
#define _COMMAND_PUBLISHER_H
#define CSPACE_RESOLUTION 0.05
using namespace nav_msgs;

typedef struct {
	geometry_msgs::Pose p;
	double orientation;
	double speed;
	double rotation;
} PoseState;



#endif	
