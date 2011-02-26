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
#include <eecs376_msgs/PathSegment.h>
#include <eecs376_msgs/PathList.h>
#include "MathyStuff.h"
#include "CSpaceFuncs.h"

#define REFRESH_RATE 10

#define LINE 1
#define CURVE 2
#define POINT_TURN 3

#define STD_TURN_RAD 1.04 //given in the assignment

#define MAX_LINEAR .5
#define MAX_ANGULAR .5
#define MAX_LINEAR_ACC .1
#define MAX_ANGULAR_ACC .1

using namespace cv;
using namespace std;
using namespace eecs376_msgs;
tf::TransformListener *tfl;

geometry_msgs::Point Point3toGeoPoint (Point3 A)
{
	geometry_msgs::Point Ref_Point;
	Ref_Point.x = A.X;
	Ref_Point.y = A.Y;
	Ref_Point.z = A.Z;
	return Ref_Point;
}

PathSegment MakeLine(Point3 A, Point3 B, int SegNum)  //woot it makes a line
{
	PathSegment P;
	P.seg_type = LINE;
	P.seg_number = SegNum;
	P.seg_length = Distance3(A,B);
	P.ref_point = Point3toGeoPoint(A);
	Point3 Vec = B-A;
	P.init_tan_angle = tf::createQuaternionMsgFromYaw(atan2(Vec.Y, Vec.X));
	P.curvature = 1337;
	
	P.max_speeds.linear.x = MAX_LINEAR;
	P.max_speeds.linear.y = 0;
	P.max_speeds.linear.z = 0;
	P.min_speeds.linear.x = 0;
	P.min_speeds.linear.y = 0;
	P.min_speeds.linear.z = 0;
	
	P.max_speeds.angular.x = 0;
	P.max_speeds.angular.y = 0;
	P.max_speeds.angular.z = MAX_ANGULAR;
	P.min_speeds.angular.x = 0;
	P.min_speeds.angular.y = 0;
	P.min_speeds.angular.z = 0;
	P.accel_limit = MAX_LINEAR_ACC; //dude wtf, why is there just one here?  is it angular or linear?
	return P;
}

PathSegment MakeTurnInPlace (geometry_msgs::Quaternion InitAngle, geometry_msgs::Quaternion FinalAngle, geometry_msgs::Point ref_point, int SegNum)
{
	double theta1 = tf::getYaw(InitAngle);
	double theta2 = tf::getYaw(FinalAngle);
	
	PathSegment P;
	P.seg_type = POINT_TURN;
	P.seg_number = SegNum;
	P.seg_length = theta2-theta1;
	P.ref_point = ref_point;
	P.init_tan_angle = InitAngle;
	P.curvature = 1337;
	
	P.max_speeds.linear.x = MAX_LINEAR;
	P.max_speeds.linear.y = 0;
	P.max_speeds.linear.z = 0;
	P.min_speeds.linear.x = 0;
	P.min_speeds.linear.y = 0;
	P.min_speeds.linear.z = 0;
	
	P.max_speeds.angular.x = 0;
	P.max_speeds.angular.y = 0;
	P.max_speeds.angular.z = MAX_ANGULAR;
	P.min_speeds.angular.x = 0;
	P.min_speeds.angular.y = 0;
	P.min_speeds.angular.z = 0;
	P.accel_limit = MAX_LINEAR_ACC; //dude wtf, why is there just one here?  is it angular or linear?
	return P;
}

PathSegment MakeCurve(geometry_msgs::Quaternion InitAngle, geometry_msgs::Quaternion FinalAngle, geometry_msgs::Point ref_point, int SegNum)
{
	
	double theta1 = tf::getYaw(InitAngle);
	double theta2 = tf::getYaw(FinalAngle);
	
	PathSegment P;
	P.seg_type = CURVE;
	P.seg_number = SegNum;
	P.seg_length = theta2-theta1;
	P.ref_point = ref_point;
	P.init_tan_angle = InitAngle;
	P.curvature = 1/STD_TURN_RAD;
	
	P.max_speeds.linear.x = MAX_LINEAR;
	P.max_speeds.linear.y = 0;
	P.max_speeds.linear.z = 0;
	P.min_speeds.linear.x = 0;
	P.min_speeds.linear.y = 0;
	P.min_speeds.linear.z = 0;
	
	P.max_speeds.angular.x = 0;
	P.max_speeds.angular.y = 0;
	P.max_speeds.angular.z = MAX_ANGULAR;
	P.min_speeds.angular.x = 0;
	P.min_speeds.angular.y = 0;
	P.min_speeds.angular.z = 0;
	P.accel_limit = MAX_LINEAR_ACC; //dude wtf, why is there just one here?  is it angular or linear?
	return P;
}

void MoveBack1(Point3 A, Point3 B, PathSegment* Segment) // moves the start and end points as special cases
{
	A = (A-B)/2.0; //just want the distance from A to the midpoint
	Segment->ref_point.x -= A.X;
	Segment->ref_point.y -= A.Y;
	Segment->seg_length +=Magnitude3(A);
}

void MoveBack2(Point3 A, Point3 B, PathSegment* Segment)
{
	Segment->seg_length +=Magnitude3(B-A)/2.0;
}

void GetCurveAndLines( Point3 A, Point3 B, Point3 C, PathSegment* FirstLine, PathSegment* Curve, PathSegment* SecondLine, int* SegNum)
{
	double Theta = Dot3(A-B, B-C)/(Magnitude3(A-B)*Magnitude3(B-C));  //impliment the math that I did earlier
	Point3 D = (A+C)/2.0;
	Point3 Center = B+(D-B)/Magnitude3(D-B) * (STD_TURN_RAD/tan(Theta/2.0));
	Point3 Bprime = A+Dot3(Center-A,B-A)*(B-A)/Magnitude3(B-A);
	Point3 Bdoubleprime = C+Dot3(Center-C,B-C)*(B-C)/Magnitude3(B-C); //the equation in the pic ben sent me is wrong I think.  C should substitute for A, not B for A and C for B like I did.
	Point3 Midpoint1 = A+(A-B)/2.0;  //midpoints are used in a sec
	Point3 Midpoint2 = C+(C-B)/2.0;
	if (Dot3(Midpoint1, B-A)<Dot3(Bprime, B-A)||Dot3(Midpoint2, B-C)<Dot3(Bdoubleprime, B-C))
	{
		(*FirstLine) = MakeLine(Midpoint1, B, (*SegNum)++);
		(*SecondLine)  = MakeLine(B,Midpoint2, (*SegNum)++);
		(*Curve) = MakeTurnInPlace(FirstLine->init_tan_angle, SecondLine->init_tan_angle, SecondLine->ref_point, (*SegNum)++) ;
	}
	else 
	{
		(*FirstLine) = MakeLine(Midpoint1, Bprime, (*SegNum)++);
		(*SecondLine) = MakeLine(Bdoubleprime, Midpoint2,(*SegNum)++);
		(*Curve) = MakeCurve(FirstLine->init_tan_angle, SecondLine->init_tan_angle, FirstLine->ref_point,(*SegNum)++) ;
	}
}

PathList insertTurns(list<Point> P)
{
	Point3* PointList = (Point3*)calloc(sizeof(Point3),P.size());  //this should be the list of points that ben's algorithm puts out
	int PointListLength = P.size();
	
	list<Point>::iterator it; 
	int i = 0;
	for (it = P.begin(); it!=P.end(); it++)
	{
		PointList[i].X = it->x;
		PointList[i].Y = it->y;
		PointList[i].Z = 0;
		i++;
	}
	
	PathList ReturnVal;
	vector<PathSegment> path = vector<PathSegment>(3*PointListLength);
	ReturnVal.path_list.assign(path.begin(), path.end());
	//(PathSegment*)malloc(sizeof(PathSegment)*(3*(PointListLength))); //the equation for this comes from the path planner splitting each segment except for the first and last.
	int SegNum = 0;
	Point3 A, B, C;
	PathSegment FirstLine, Curve, SecondLine;

	
	for (int i = 0; i<PointListLength-1; i++)
	{
		A = PointList[i];
		B = PointList[i+1];
		C = PointList[i+2];
		
		GetCurveAndLines(A, B, C, &FirstLine, &Curve, &SecondLine, &SegNum);//hand the points A,B,C to the curve maker thing
		if(i == 0)
		{
			MoveBack1(A,B, &FirstLine);
		}
		else if (i == PointListLength-2)
		{
			MoveBack2(B,C, &SecondLine);
		}
		
		ReturnVal.path_list[3*i] = FirstLine;
		ReturnVal.path_list[3*i+1] = Curve;
		ReturnVal.path_list[3*i+2] = SecondLine;
	}
	return ReturnVal;
}

list<Point> bugAlgorithm(Mat_<bool>* map_p, Point dest, geometry_msgs::PoseStamped start, geometry_msgs::Pose origin) {
	list<Point> path =  list<Point>();
	Mat_<bool> map = *map_p;
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
	double distances[] = {3.3,12.2,4,-1};
	int i = 0;
	double distance = distances[0];
	while((fabs(x - dest.x) > 0.5 || fabs(y - dest.y) > 0.5) && points++ < 200) {
		if(distance < 0.001) {
			i++;
			if(i >= 3) {
				break;
			}
			heading -= 3.14159/2;
			if(heading < -2 * 3.14159) {
				heading += 2 * 3.14159;
			}
			distance = distances[i];
		}
		x = x + CSPACE_RESOLUTION * cos(heading);
		y = y + CSPACE_RESOLUTION * sin(heading);
		distance -= CSPACE_RESOLUTION;	
		double distance_to_check = avoiding ? 0.6 : 0.75;
		wallx = x + distance_to_check * cos(heading - 3.14159/2);
		wally = y + distance_to_check * sin(heading - 3.14159/2);
		int grid_wall_x = (int)((wallx-origin.position.x)/CSPACE_RESOLUTION);
		int grid_wall_y = (int)((wally-origin.position.y)/CSPACE_RESOLUTION);
		int grid_x = (int)((x-origin.position.x)/CSPACE_RESOLUTION);
		int grid_y = (int)((y-origin.position.y)/CSPACE_RESOLUTION);
		
		if(!map(grid_wall_x, grid_wall_y)) {
			
			if(!avoiding) {
				//this means that we need to turn
			/**	path.push_back(Point(x,y));
				//now move us around the circle
				heading -= 3.14159/2;
				x += 0.75 * cos(heading);
				y += 0.75 * sin(heading);
				distance -= 0.75;
			*/
			} else {
				//this means we need to readjust to go back 2 feet
				
				path.push_back(Point(x,y));
				x = wallx + 1.5 * cos(heading);
				y = wally + 1.5 * cos(heading);
				path.push_back(Point(x,y));
				avoiding=false;
				distance -= 1.5;
			}
		} else if(map(grid_x, grid_y) && map(grid_x+1,grid_y) && map(grid_x-1,grid_y+1) && map(grid_x+1,grid_y+1)) {
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
geometry_msgs::Pose goalPose; 
geometry_msgs::Pose mapOrigin;
geometry_msgs::PoseStamped poseActual;
bool LIDARcalled = false;
bool poseDescalled = false;
bool goalPosecalled = false;
bool poseActualcalled = false;
void LIDAR_Callback(const boost::shared_ptr<nav_msgs::OccupancyGrid  const>& LIDAR_Map)
{
	lastLIDAR_Map = getMap(*LIDAR_Map);
	mapOrigin = (*LIDAR_Map).info.origin;
	LIDARcalled = true;
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
void poseActual_Callback(const geometry_msgs::PoseStamped::ConstPtr& newPoseActual) 
{
	poseActual = *newPoseActual;
	poseActualcalled=true;
}
void poseDes_Callback(const geometry_msgs::PoseStamped::ConstPtr& newPoseDes)
{
	poseDes = *newPoseDes;
	poseDescalled = true;
}
void goalPose_Callback(const geometry_msgs::Pose::ConstPtr& newGoalPose)
{
	goalPose = *newGoalPose;
	goalPosecalled = true;
}



int main(int argc,char **argv)
{
	cout<<"3\n";
	ros::init(argc,argv,"pathPlanner");//name of this node
	tfl = new tf::TransformListener();
      	double amount_to_change = 0.0;    
      	cout<<"3\n";
	ros::NodeHandle n;
	//ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Publisher path_pub = n.advertise<eecs376_msgs::PathList>("pathList",10);
	ros::Subscriber sub1 = n.subscribe<nav_msgs::OccupancyGrid>("LIDAR_Map", 10, LIDAR_Callback); 
	//ros::Subscriber sub2 = n.subscribe<cv::Mat>("SONAR_Map", 1, SONAR_Callback); 
	//ros::Subscriber sub3 = n.subscribe<cv::Mat>("VISION_Map", 1, VISION_Callback); 
	ros::Subscriber sub4 = n.subscribe<geometry_msgs::PoseStamped>("poseDes", 10, poseDes_Callback);
	ros::Subscriber sub5 = n.subscribe<geometry_msgs::Pose>("goalPose", 10, goalPose_Callback);
	cout<<"3\n";
	ros::Duration elapsed_time; // define a variable to hold elapsed time
	ros::Rate naptime(REFRESH_RATE); //will perform sleeps to enforce loop rate of "10" Hz
	while (!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly. Wait until ros::Time::now() will be valid, but let any callbacks happen
      	cout<<"3\n";
	ros::Time birthday = ros::Time::now();
	//desired_pose.header.stamp = birthday;
	while (!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce(); // wait until there is transform data available before starting our controller loopros::Time birthday= ros::Time::now(); // get the current time, which defines our start time, called "birthday"
	cout<<"3\n";
	ROS_INFO("birthday started as %f", birthday.toSec());
  
	while (ros::ok()) // do work here
	{
	//cout<<"3\n";
		ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely
		ros::Time current_time = ros::Time::now();
		//desired_pose.header.stamp = current_time;
		elapsed_time= ros::Time::now()-birthday;
	//	ROS_INFO("birthday is %f", birthday.toSec());
	//	ROS_INFO("elapsed time is %f", elapsed_time.toSec());

	//cout<<"3\n";
		if(LIDARcalled  && goalPosecalled) {
			if(!poseDescalled) {
				//this means we haven't used yet, so use our actual pose
				poseDes.pose.position.x = 7.57;
				poseDes.pose.position.y = 14.26;
				poseDes.pose.orientation =  tf::createQuaternionMsgFromYaw(-2.354);
			}
			list<Point> points = bugAlgorithm(lastLIDAR_Map, Point(goalPose.position.x, goalPose.position.y),poseDes, mapOrigin);	
			PathList turns = insertTurns(points);
			path_pub.publish(turns);
			cout<<"3published"<<"\n";	
		}
		else
		{
			if(!LIDARcalled)cout<<"No LIDAR\n";
			if(!poseDescalled)cout<<"No poseDes\n";
			if(!goalPosecalled)cout<<"No goalPose\n";
		}
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (100ms) period
		//thus enforcing that we achieve the desired update rate (10Hz)
	}
	return 0; // this code will only get here if this node was told to shut down, which is
	// reflected in ros::ok() is false 
}
