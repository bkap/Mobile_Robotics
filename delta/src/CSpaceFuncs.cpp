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
#include <visualization_msgs/Marker.h>
#include <vector>
#ifndef CSPACEFUNCS_cpp
#define CSPACEFUNCS_cpp
using namespace std;
using namespace cv;
using namespace std;
/**assume orientation and resolution are the same*/
Mat_<bool>* getMap(const nav_msgs::OccupancyGrid& grid) {
	Mat_<bool>* m = new Mat_<bool>(grid.info.width, grid.info.height);
	for(unsigned int i = 0; i < grid.info.height; i ++) {
		for(unsigned int j = 0; j < grid.info.width; j++) {
			(*m)(i,j) = (grid.data[i * grid.info.width + j] > 10);
			if((*m)(i,j)) {
				//cout << "stuff happens"<< endl;
			}
		}
	}
	return m;

}

void PlotMap(list<geometry_msgs::Point> PointsToPlot, ros::Publisher *vis_pub, float r, float g, float b, float cubesize)
{
	//some code taken directly from the wiki and then modified
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = cubesize;
	marker.scale.y = cubesize;
	marker.scale.z = cubesize;
	marker.color.a = 0.0;
	marker.color.r =  r;
	marker.color.g = g;
	marker.color.b = b;
	
	/*
	geometry_msgs::Point* RetVal = (*geometry_msgs::Point)malloc(PointsToPlot.size() * sizeof(geometry_msgs::Point));
	*/
	
	vector<geometry_msgs::Point>* RetVal = new vector<geometry_msgs::Point>();
	while(!PointsToPlot.empty())
	{
		geometry_msgs::Point P = PointsToPlot.back();
		PointsToPlot.pop_back();
		RetVal->push_back(P);
		
	}
	marker.points = *RetVal;
	//marker.colors = NULL;  //the wiki says that this should 
	
	vis_pub->publish( marker );

}

#endif
