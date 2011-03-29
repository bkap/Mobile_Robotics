#include "command_publisher.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include<tf/transform_listener.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <list>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <eecs376_msgs/PathSegment.h>
#include <eecs376_msgs/PathList.h>
#include <visualization_msgs/Marker.h>
#include "MathyStuff.h"
#include "CSpaceFuncs.h"

#define REFRESH_RATE 10

//define the 3 types of segments, these need to be consistent with dpcrawler
#define LINE 1
#define CURVE 2
#define POINT_TURN 3

#define STD_TURN_RAD .6 //given in the assignment but changed because it seemed to work better

//define maximum speeds and accelerations
#define MAX_LINEAR .5
#define MAX_ANGULAR .5
#define MAX_LINEAR_ACC .1
#define MAX_ANGULAR_ACC .1

//uses some namespaces
using namespace cv;
using namespace std;
using namespace eecs376_msgs;
tf::TransformListener *tfl; //transforms crap into better frames

//converts between the Point3 class and a geometry point for use in messages
geometry_msgs::Point Point3toGeoPoint (Point3 A)
{
	geometry_msgs::Point Ref_Point;
	Ref_Point.x = A.X;
	Ref_Point.y = A.Y;
	Ref_Point.z = A.Z;
	return Ref_Point;
}

//makes a line from A to B of a given SegNum
PathSegment MakeLine(Point3 A, Point3 B, int SegNum)  //woot it makes a line
{
	PathSegment P;
	P.seg_type = LINE;
	P.seg_number = SegNum;
	//cout << "makeline" << endl;
	//cout << A.X << "," << A.Y << ":" << B.X << "," << B.Y << endl;
	P.seg_length = Distance3(A,B);
	//cout<< P.seg_length << endl;
	P.ref_point = Point3toGeoPoint(A);
	Point3 Vec = B-A;
	P.init_tan_angle = tf::createQuaternionMsgFromYaw(atan2(Vec.Y, Vec.X));
	P.curvature = 1337;
	
	P.max_speeds.linear.x = MAX_LINEAR*0.5;
	P.max_speeds.linear.y = 0;
	P.max_speeds.linear.z = 0;
	P.min_speeds.linear.x = 0;
	P.min_speeds.linear.y = 0;
	P.min_speeds.linear.z = 0;
	
	P.max_speeds.angular.x = 0;
	P.max_speeds.angular.y = 0;
	P.max_speeds.angular.z = MAX_ANGULAR*2.0;
	P.min_speeds.angular.x = 0;
	P.min_speeds.angular.y = 0;
	P.min_speeds.angular.z = 0;
	P.accel_limit = MAX_LINEAR_ACC; //dude wtf, why is there just one here?  is it angular or linear?
	return P;
}

//generate a turn in place given an initial heading, a final heading, and a point to turn on.
PathSegment MakeTurnInPlace (double InitAngle, double FinalAngle, geometry_msgs::Point ref_point, int SegNum)
{
	double theta1 = InitAngle;
	double theta2 = FinalAngle;
	
	PathSegment P;
	P.seg_type = POINT_TURN;
	P.seg_number = SegNum;
	P.seg_length = fabs(theta2-theta1); //how much to turn
	P.ref_point = ref_point; //where to turn
	P.init_tan_angle = tf::createQuaternionMsgFromYaw(InitAngle); //initial heading as a quaternion
	P.curvature = (FinalAngle>InitAngle)?1:-1;//direction to turn
	
	P.max_speeds.linear.x = 0;
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

//makes a curve again initial and final headings as well as start and end points
PathSegment MakeCurve(double InitAngle, double FinalAngle, int SegNum, Point3 A, Point3 B)
{
	Point3 M = (A+B)/2.0;  //midpoint
	Point3 MA = M-A;  //vector from midpoint to A
	Point3 Center = M - Point3(MA.Y, -MA.X, 0)/tan((FinalAngle-InitAngle)/2.0);//get the center point.  derived using basic trig
	double Radius = Distance3(A, Center); //this may or may not work

	PathSegment P;
	P.seg_type = CURVE;
	P.seg_number = SegNum;
	P.seg_length = FinalAngle-InitAngle;
		if(P.seg_length > 3.14159) {
		P.seg_length -= 2 * 3.14159;
	} else if(P.seg_length < -3.14159) {
		P.seg_length += 2 * 3.14159;
	}

	P.curvature = 1/STD_TURN_RAD;// I know that curvature should be 1/Radius, but that didn't work for some reason so it was overriden
	P.curvature *= (P.seg_length>0)?1:-1;
	P.seg_length = fabs(P.seg_length);

	geometry_msgs::Point gmPoint;
	gmPoint.x = Center.X;
	gmPoint.y = Center.Y;
	gmPoint.z = 0.0f;
	P.ref_point = gmPoint;
	P.init_tan_angle = tf::createQuaternionMsgFromYaw(InitAngle);
	
	
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

//these two functions were used in the GetCurveAndLines routine.  defunct.

void MoveBack1(Point3 A, Point3 B, PathSegment* Segment) // moves the start and end points as special cases
{
	A = (A-B)/2.0; //just want the distance from A to the midpoint
	Segment->ref_point.x += A.X;
	Segment->ref_point.y += A.Y;
	Segment->seg_length +=Magnitude3(A);
}

void MoveBack2(Point3 A, Point3 B, PathSegment* Segment)
{
	Segment->seg_length +=Magnitude3(B-A)/2.0;
}


this was designed for use in the insertTurns function.  became defunct when insert turns was merged into planner

void GetCurveAndLines( Point3 A, Point3 B, Point3 C, PathSegment* FirstLine, PathSegment* Curve, PathSegment* SecondLine, int* SegNum)
{

	find the points needed to genereate 2 lines with a curve in between
	double Theta = Dot3(A-B, B-C)/(Magnitude3(A-B)*Magnitude3(B-C));  //impliment the math that I did earlier
	Point3 D = (A+C)/2.0;
	Point3 Center = B+(D-B)/Magnitude3(D-B) * (STD_TURN_RAD/acos(tan(Theta/2.0)));
	Point3 Bprime = A+Dot3(Center-A,B-A)*(B-A)/Magnitude3(B-A);

	Point3 Bdoubleprime = B+Dot3(Center-C,B-C)*(B-C)/Magnitude3(B-C); //the equation in the pic ben sent me is wrong I think.  C should substitute for A, not B for A and C for B like I did.
	Point3 Midpoint1 = A+(A-B)/2.0;  //midpoints are used in a sec
	Point3 Midpoint2 = C-(C-B)/2.0;
	if (Dot3(Midpoint1, B-A)<Dot3(Bprime, B-A)||Dot3(Midpoint2, B-C)<Dot3(Bdoubleprime, B-C))
	{
		(*FirstLine) = MakeLine(Midpoint1, B, (*SegNum)++);
		(*SecondLine)  = MakeLine(B,Midpoint2, (*SegNum)+1);
		(*Curve) = MakeTurnInPlace(FirstLine->init_tan_angle, SecondLine->init_tan_angle, SecondLine->ref_point, (*SegNum)++) ;
		(*SegNum)++;
	}
	else 
	{
		(*FirstLine) = MakeLine(Midpoint1, Bprime, (*SegNum)++);
		(*SecondLine) = MakeLine(Bdoubleprime, Midpoint2,(*SegNum)+1);
		(*Curve) = MakeCurve(FirstLine->init_tan_angle, SecondLine->init_tan_angle, FirstLine->ref_point,(*SegNum)++) ;
		(*SegNum)++;
	}
}

// this was supposed to take a list of points and turn them into a series of lines and turns, following the pattern (line, turn, line, line, turn, line ...) note that any two adjacent lines are parallel and connect at start and end points to essentially make a single larger line segment.  

//this function was merged with parts of the bug algorithm because we only needed the special cases of 90 and 45 degree turns. in the future, this will be included because it is much more generically written

PathList insertTurns(list<Point2d> P)
{
	//convert from the list<Point2d> to an array of Point3's
	Point3* PointList = (Point3*)calloc(sizeof(Point3),P.size());  //this should be the list of points that ben's algorithm puts out
	int PointListLength = P.size();
	
	list<Point2d>::iterator it; 
	int i = 0;
	for (it = P.begin(); it!=P.end(); it++)
	{
		PointList[i].X = it->x;
		PointList[i].Y = it->y;
		PointList[i].Z = 0;
		i++;
	}
	
	PathList ReturnVal;//the path list that we will eventually return
	vector<PathSegment> path = vector<PathSegment>(3*(PointListLength-2));//3(n-2) segments are needed when using the line turn line line turn line pattern
	ReturnVal.path_list.assign(path.begin(), path.end());
	//(PathSegment*)malloc(sizeof(PathSegment)*(3*(PointListLength))); //the equation for this comes from the path planner splitting each segment except for the first and last.
	int SegNum = 0;
	Point3 A, B, C; //points A,B,C for the line turn line pattern
	PathSegment FirstLine, Curve, SecondLine; //the path segments we will generate

	
	for (int i = 0; i<PointListLength-2; i++)
	{
		A = PointList[i];  //copy to temp variables for readability
		B = PointList[i+1];
		C = PointList[i+2];
		
		//from the points (A,B,C), generate (line, turn, line)
		GetCurveAndLines(A, B, C, &FirstLine, &Curve, &SecondLine, &SegNum);//hand the points A,B,C to the curve maker thing
		
		//the first and last line segments need to be special cased.  the move back functions just add to their length 
		if(i == 0)
		{
			MoveBack1(A,B, &FirstLine);
		}
		else if (i == PointListLength-3)
		{
			MoveBack2(B,C, &SecondLine);
		}
		
		ReturnVal.path_list[3*i] = FirstLine;
		ReturnVal.path_list[3*i+1] = Curve;
		ReturnVal.path_list[3*i+2] = SecondLine;
	}
	return ReturnVal;//return the pathlist
}

//finds a point along a curve given inital parameters
Point3 findPointAlongCircle(Point3 startPoint, double initial_heading, double change_in_heading, double radius) {
		double heading = initial_heading;
		if(change_in_heading > 0) {
			//we are going positive angle, add 90
			heading += 3.14159 / 2;
		} else {
			heading -= 3.14159/2;
		}
		Point3 center = Point3(startPoint.X + cos(heading) * radius, startPoint.Y + sin(heading) * radius,0.0);
		//now invert heading and adjust it by by change_in_heading
		heading = heading - 3.14159 + change_in_heading;
		return Point3(center.X + cos(heading) * radius, center.Y + sin(heading) * radius, 0.0);
}


using namespace std;

cv::Mat_<bool> *lastCSpace_Map;
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
void LIDAR_Callback(const boost::shared_ptr<nav_msgs::OccupancyGrid  const>& CSpace_Map)
{
	//cout << "recieved width: " <<  (*CSpace_Map).info.width<< endl;
	if(lastCSpace_Map != NULL) {
		delete lastCSpace_Map;
	}
	lastCSpace_Map = getMap(*CSpace_Map);
	mapOrigin = (*CSpace_Map).info.origin;
	LIDARcalled = true;
}
//MOAR CALLBACKS!!!!!!!!!!
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

	ros::Publisher path_pub = n.advertise<eecs376_msgs::PathList>("pathList",10);
	ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",10);
	ros::Subscriber sub1 = n.subscribe<nav_msgs::OccupancyGrid>("CSpace_Map", 10, LIDAR_Callback); 

	ros::Subscriber sub4 = n.subscribe<geometry_msgs::PoseStamped>("poseDes", 10, poseDes_Callback);
	ros::Subscriber sub5 = n.subscribe<geometry_msgs::Pose>("goalPose", 10, goalPose_Callback);
	cout<<"3\n";
	ros::Duration elapsed_time; // define a variable to hold elapsed time
	ros::Rate naptime(REFRESH_RATE); //will perform sleeps to enforce loop rate of "10" Hz
	while (ros::ok()&&!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly. Wait until ros::Time::now() will be valid, but let any callbacks happen
      	cout<<"3\n";
	ros::Time birthday = ros::Time::now();
	//desired_pose.header.stamp = birthday;
	while (ros::ok()&&!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce(); // wait until there is transform data available before starting our controller loopros::Time birthday= ros::Time::now(); // get the current time, which defines our start time, called "birthday"
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
		if(LIDARcalled && goalPosecalled) {
			if(!poseDescalled) {
				//cout<<"yo1\n";
				//this means we haven't used yet, so use our actual pose
				poseDes.pose.position.x = 7.57;
				poseDes.pose.position.y = 14.26;
				poseDes.pose.orientation =  tf::createQuaternionMsgFromYaw(-2.361);
				//cout<<"yo2\n";
			}
		
			list<geometry_msgs::Point> points;
			geometry_msgs::Point p;
			for(int i=0;i<lastCSpace_Map->size().height; i++)
			{
				for(int j =0; j<lastCSpace_Map->size().width; j++)
				{
					if((*lastCSpace_Map)(i,j)) {
						p.x = mapOrigin.position.x+.05*j;
						p.y = mapOrigin.position.y*.05*i;
						points.push_back(p); 
					}
				}
			}
			PathList turns = bugAlgorithm(lastCSpace_Map, Point2d(goalPose.position.x, goalPose.position.y),poseDes, mapOrigin);
			PlotMap(points, &vis_pub, 0.0,1.0,0.0, .05);
			//cout<<"publishing\n";
			path_pub.publish(turns);
			//cout<<"3published"<<"\n";	
		}
		else
		{
			if(!LIDARcalled)cout<<"No LIDAR\n";
			if(!poseDescalled)cout<<"No poseDes\n";
			if(!goalPosecalled)cout<<"No goalPose\n";
			//if(!poseActualcalled)cout<<"No poseActual\n";
		}
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (100ms) period
		//thus enforcing that we achieve the desired update rate (10Hz)
	}
	return 0; // this code will only get here if this node was told to shut down, which is
	// reflected in ros::ok() is false 
}
