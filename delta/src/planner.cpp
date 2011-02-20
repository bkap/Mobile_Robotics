#include "command_publisher.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gemoetry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include<tf/transform_listener.h>
#include "cv.h"
#include <list>
#include <math.h>
#include <algorithm>
#include <iostream>

using namespace cv;
/**assume orientation and resolution are the same*/
Mat& getMap(OccupancyGrid grid) {
	Mat<bool> m = Mat<int>(grid.info.width, grid.info.height);
	for(int i = 0; i < grid.info.height; i ++) {
		for(int j = 0; j < grid.info.width; j++) {
			m(i,j) = (grid.data[i * grid.info.width + j] > 10);
		}
	}
	return m;

}

list<Pose> bugAlgorithm(Mat& cspace, Point_ dest, PoseStamped start) {
	list<Pose> path =  list<Point_>();
	//go forward until the space 75 cm to the right of the robot is clear. FInd
	//location
	double heading = tf::GetYaw(start.pose.orientation);
	double x = start.pose.x;
	double y = start.pose.y;
	double wallx, wally;
	bool avoiding = false;
	//head forward until you can turn or until you hit the entrance
	while(fabs(x - dest.x) >= 0.03 || fabs(y - dest.y) >= 0.03) {
		x += 0.05 * cos(heading);
		y += 0.05 * sin(heading);
		
		double distance_to_check = avoiding ? 0.6 : 0.75;
		wallx = x + distance_to_check * cos(heading - 3.14159/2);
		wally = y + distance_to_check * sin(heading - 3.14159/2);
		int grid_wall_x = (int)(wallx/CSPACE_RESOLUTION);
		int grid_wall_y = (int)(wally/CSPACE_RESOLUTION);
		int grid_x = (int)(x/CSPACE_RESOLUTION);
		int grid_y = (int)(y/CSPACE_RESOLUTION);
		if(!map(grid_wall_x, grid_wall_y)) {
			if(!avoiding) {
				//this means that we need to turn
				path.push_back(Pose(x,y,heading));
				//now move us around the circle
				x += 0.75 * cos(heading);
				y += 0.75 * sin(heading);
				heading -= 3.14159/2;
				x += 0.75 * cos(heading);
				y += 0.75 * sin(heading);
				path.push-back(Pose(x,y,heading));
			} else {
				//this means we need to readjust to go back 2 feet
				path.push_back(Pose(x,y,heading));
				x = wallx + 1.5 * cos(heading);
				y = wally + 1.5 * cos(heading);
				path.push_back(Pose(x,y,heading));
				avoiding=false;
			}
		} else if(map(grid_x, grid_y)) {
			//we have an obstacle. Back up 1.5 m and swerve around
		
			//TODO: check to see if we are in fact traveling 1.5m before
			//turning
			double prev_turn_x = x - 1.5 * cos(heading);
			double prev_turn_y = y - 1.5 * sin(heading);
			path.push_back(Pose(prev_turn_x, prev_turn_y,heading));
			
			//now get the final position
			x += 0.6 * cos(heading + 3.14159/2);
			y += 0.6 * sin(heading + 3.14159/2);
			path.push_back(Pose(x,y,heading));
			avoiding = true;
		}
	}
	path.push_back(Pose(x,y,heading));
	return path;
}


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
