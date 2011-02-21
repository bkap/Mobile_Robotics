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
#define REFRESH_RATE 0.1
using namespace cv;
using namespace std;
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




list<Point> bugAlgorithm(Mat_<bool>& map, Point dest, geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped origin) {
	list<Point> path =  list<Point>();
	//go forward until the space 75 cm to the right of the robot is clear. FInd
	//location
	double heading = tf::getYaw(start.pose.orientation);
	double x = start.pose.position.x;
	double y = start.pose.position.y;
	double wallx, wally;
	bool avoiding = false;
	path.push_back(Point(x,y));
	//head forward until you can turn or until you hit the entrance
	while(fabs(x - dest.x) > 0.5 || fabs(y - dest.y) > 0.5) {
		x = x + CSPACE_RESOLUTION * cos(heading);
		y = y + CSPACE_RESOLUTION * sin(heading);
		
		double distance_to_check = avoiding ? 0.6 : 0.75;
		wallx = x + distance_to_check * cos(heading - 3.14159/2);
		wally = y + distance_to_check * sin(heading - 3.14159/2);
		int grid_wall_x = (int)((wallx-origin.pose.position.x)/CSPACE_RESOLUTION);
		int grid_wall_y = (int)((wally-origin.pose.position.y)/CSPACE_RESOLUTION);
		int grid_x = (int)((x-origin.pose.position.x)/CSPACE_RESOLUTION);
		int grid_y = (int)((y-origin.pose.position.y)/CSPACE_RESOLUTION);
		
		if(!map(grid_wall_x, grid_wall_y)) {
			
			if(!avoiding) {
				//this means that we need to turn
				path.push_back(Point(x,y));
				//now move us around the circle
				heading -= 3.14159/2;
				x += 0.75 * cos(heading);
				y += 0.75 * sin(heading);
			} else {
				//this means we need to readjust to go back 2 feet
				path.push_back(Point(x,y));
				x = wallx + 1.5 * cos(heading);
				y = wally + 1.5 * cos(heading);
				path.push_back(Point(x,y));
				avoiding=false;
			}
		} else if(map(grid_x, grid_y)) {
			//we have an obstacle. Back up 1.5 m and swerve around
		
			//TODO: check to see if we are in fact traveling 1.5m before
			//turning
			double prev_turn_x = x - 1.5 * cos(heading);
			double prev_turn_y = y - 1.5 * sin(heading);
			path.push_back(Point(prev_turn_x, prev_turn_y));
			
			//now get the final position
			x += 0.6 * cos(heading + 3.14159/2);
			y += 0.6 * sin(heading + 3.14159/2);
			path.push_back(Point(x,y));
			avoiding = true;
		}
	}
	path.push_back(Point(x,y));
	return path;
}


using namespace std;

cv::Mat_<bool> *lastLIDAR_Map;
cv::Mat_<bool> *lastVISION_Map;
cv::Mat_<bool> *lastSONAR_Map;
geometry_msgs::PoseStamped poseDes;
geometry_msgs::PoseStamped goalPose; 
geometry_msgs::Pose mapOrigin;
void LIDAR_Callback(const boost::shared_ptr<nav_msgs::OccupancyGrid  const>& LIDAR_Map)
{
	lastLIDAR_Map = getMap(*LIDAR_Map);
	mapOrigin = (*LIDAR_Map).info.origin;
}
/*
void SONAR_Callback(const boost::shared_ptr<cv::Mat  const>& SONAR_Map)
{
	lastSONAR_Map = *SONAR_Map;
}

void VISION_Callback(const boost::shared_ptr<cv::Mat  const>& VISION_Map)
{
	lastVISION_Map = *VISION_Map;
}
*/
void poseDes_Callback(const geometry_msgs::PoseStamped::ConstPtr& newPoseDes)
{
	poseDes = *newPoseDes;
}
void goalPose_Callback(const geometry_msgs::PoseStamped::ConstPtr& newGoalPose)
{
	goalPose = *newGoalPose;
}



int main(int argc,char **argv)
{
	
	ros::init(argc,argv,"pathPlanner");//name of this node
	 
      	double amount_to_change = 0.0;    
      		
	ros::NodeHandle n;
	//ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Publisher des_pose_pub = n.advertise<geometry_msgs::PoseStamped>("poseDes", 1);
	
	ros::Subscriber sub1 = n.subscribe<nav_msgs::OccupancyGrid>("LIDAR_Map", 1, LIDAR_Callback); 
	//ros::Subscriber sub2 = n.subscribe<cv::Mat>("SONAR_Map", 1, SONAR_Callback); 
	//ros::Subscriber sub3 = n.subscribe<cv::Mat>("VISION_Map", 1, VISION_Callback); 
	ros::Subscriber sub4 = n.subscribe<geometry_msgs::PoseStamped>("poseDes", 1, poseDes_Callback);
	ros::Subscriber sub5 = n.subscribe<geometry_msgs::PoseStamped>("goalPose", 1, goalPose_Callback);

	ros::Duration elapsed_time; // define a variable to hold elapsed time
	ros::Rate naptime(1/REFRESH_RATE); //will perform sleeps to enforce loop rate of "10" Hz
	while (!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly. Wait until ros::Time::now() will be valid, but let any callbacks happen
      
	ros::Time birthday = ros::Time::now();
	desired_pose.header.stamp = birthday;
	while (!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce(); // wait until there is transform data available before starting our controller loopros::Time birthday= ros::Time::now(); // get the current time, which defines our start time, called "birthday"
	
	ROS_INFO("birthday started as %f", birthday.toSec());
  
	while (ros::ok()) // do work here
	{
		ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely
		ros::Time current_time = ros::Time::now();
		desired_pose.header.stamp = current_time;
		elapsed_time= ros::Time::now()-birthday;
		ROS_INFO("birthday is %f", birthday.toSec());
		ROS_INFO("elapsed time is %f", elapsed_time.toSec());
	
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (100ms) period
		//thus enforcing that we achieve the desired update rate (10Hz)
	}
	return 0; // this code will only get here if this node was told to shut down, which is
	// reflected in ros::ok() is false 
}
