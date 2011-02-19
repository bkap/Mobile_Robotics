#include<ros/ros.h>
#include<geometry_msgs/Twist.h> //data type for velocities
#include<math.h>
#include <iostream>
#include <algorithm>
#include<geometry_msgs/PoseStamped.h> //data type for Pose combined with frame and timestamp
#include<nav_msgs/Odometry.h> //data type for odometry information (see available fields with 'rosmsg show nav_msgs/Odometry')
#include<tf/transform_datatypes.h> // for tf::getYaw
#include<tf/transform_listener.h> // for the TransformListener class that abstracts away a lot of tf
#include "cv.h"

using namespace std;

cv::Mat lastLIDAR_Map;
cv::Mat lastVISION_Map;
cv::Mat lastSONAR_Map;
geometry_msgs::PoseStamped poseDes;
geometry_msgs::PoseStamped goalPose; 

void LIDAR_Callback(const boost::shared_ptr<cv::Mat  const>& LIDAR_Map)
{
	lastLIDAR_Map = *LIDAR_Map;
}

void SONAR_Callback(const boost::shared_ptr<cv::Mat  const>& SONAR_Map)
{
	lastSONAR_Map = *SONAR_Map;
}

void VISION_Callback(const boost::shared_ptr<cv::Mat  const>& VISION_Map)
{
	lastVISION_Map = *VISION_Map;
}

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
	
	ros::Subscriber sub1 = n.subscribe<cv::Mat>("LIDAR_Map", 1, LIDAR_Callback); 
	ros::Subscriber sub2 = n.subscribe<cv::Mat>("SONAR_Map", 1, SONAR_Callback); 
	ros::Subscriber sub3 = n.subscribe<cv::Mat>("VISION_Map", 1, VISION_Callback); 
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