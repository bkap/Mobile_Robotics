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

#include "MathyStuff.h"
#include "CSpaceFuncs.h"

#define REFRESH_RATE 0.1

using namespace cv;
using namespace std;


void GetCurveAndLines( Point3 A, Point3 B, Point3 C, PathSegment* FirstLine, PathSegment* Curve, PathSegment* SecondLine)
{
	double Theta = Dot3(A-B, B-C)/(Magnitude3(A-B)*Magnitude3(B-C));
	Point3 D = (A+C)/2.0;
	Point3 Center = B+(D-B)/Magnitude3(D-B)*STD_RADIUS/tan(Theta/2.0);
	Point3 Bprime = A+Dot3(Center-A,B-A)*(B-A)/Magnitude3(B-A);
	Point3 Bdoubleprime = C+
}

PathList insertTurns(list<Point> P)
{
	Point3* PointList = calloc(P.size, sizeof(Point3));  //this should be the list of points that ben's algorithm puts out
	int PointListLength = P.size;
	
	list<Point>::iterator it; 
	i = 0;
	for (it = P.begin; it!=p.end; it++)
	{
		PointList[i].X = it->x;
		PointList[i].Y = it->y;
		PointList[i].Z = 0;
		i++;
	}
	
	PathList ReturnVal;
	ReturnVal.path_list = malloc(sizeof(PathSegment)*(3*(PathListLength))); //the equation for this comes from the path planner splitting each segment except for the first and last.
	int SegNum = 0;
	Point3 A, B, C;
	PathSegment FirstLine, Curve, SecondLine;
	for (int i = 0; i<PointListLength-1; i++)
	{
		GetCurveAndLines(A, B, C, &FirstLine, &Curve, &SecondLine);//hand the points A,B,C to the curve maker thing
		if(i == 0)
		{
			MoveBack(A,B, &FirstLine);
		}
		else if (i == PointListLength-2)
		{
			MoveBack(B,C, &SecondLine);
		}
		
		ReturnVal.path_list[3*i] = FirstLine;
		ReturnVal.path_list[3*i+1] = Curve;
		ReturnVal.path_list[3*i+2] = SecondLine;
	}
	
	Flatten(ReturnVal.path_list);
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
	int points = 0;
	//head forward until you can turn or until you hit the entrance
	//also, stop at 200 points (10 meters) so we don't take too long
	while((fabs(x - dest.x) > 0.5 || fabs(y - dest.y) > 0.5) && points++ < 200) {
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
