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
tf::TransformListener *tfl;

// Converts a Point3f to geometry_msgs::Point
geometry_msgs::Point convertPoint3fToGeoPoint(Point3f p3f)
{
    geometry_msgs::Point geopt;
    geopt.x = p3f.x;
    geopt.y = p3f.y;
    geopt.z = p3f.z;
    return geopt;
}

Point3f convertGeoPointToPoint3f(geometry_msgs::Point geopt)
{
    Point3f p3f;
    p3f.x = geopt.x;
    p3f.y = geopt.y;
    p3f.z = geopt.z;
    return p3f;
}

Point3f convertGeoPointToPoint3f(geometry_msgs::Point32 geopt)
{
    Point3f p3f;
    p3f.x = geopt.x;
    p3f.y = geopt.y;
    p3f.z = geopt.z;
    return p3f;
}

// Get distance between two points
float getDistance(Point3f A, Point3f B)
{
    return sqrt( (A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z) );
}

float getDistance(Point3f A)
{
    return sqrt( A.x*A.x + A.y*A.y + A.z*A.z );
}

float dotProduct(Point3f A, Point3f B)
{
    return A.x*B.x + A.y*B.y + A.z*B.z;
}

//makes a line from A to B of a given SegNum
PathSegment MakeLine(Point3f A, Point3f B, int SegNum)  //woot it makes a line
{
	PathSegment P;
	P.seg_type = LINE;
	P.seg_number = SegNum;
	//cout << "makeline" << endl;
	//cout << A.X << "," << A.Y << ":" << B.X << "," << B.Y << endl;
	P.seg_length = getDistance(A, B);
	//cout<< P.seg_length << endl;
	P.ref_point = convertPoint3fToGeoPoint(A);
	Point3f vec = B-A;
	P.init_tan_angle = tf::createQuaternionMsgFromYaw(atan2(vec.y, vec.x));
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
	P.accel_limit = MAX_LINEAR_ACC;
	return P;
}

//generate a turn in place given an initial heading, a final heading, and a point to turn on.
PathSegment MakeTurnInPlace (double InitAngle, double FinalAngle, Point3f ref_point, int SegNum)
{
	double theta1 = InitAngle;
	double theta2 = FinalAngle;
	
	PathSegment P;
	P.seg_type = POINT_TURN;
	P.seg_number = SegNum;
	P.seg_length = fabs(theta2-theta1); //how much to turn
	P.ref_point = convertPoint3fToGeoPoint(ref_point); //where to turn
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
	P.accel_limit = MAX_LINEAR_ACC;
	return P;
}

//makes a curve again initial and final headings as well as start and end points
PathSegment MakeCurve(double InitAngle, double FinalAngle, Point3f A, Point3f B, int SegNum)
{
	Point3f M = (A+B)*(0.5);  //midpoint
	Point3f MA = M-A;  //vector from midpoint to A
	Point3f Center = M - Point3f(MA.y, -MA.y, 0)*(1.0/tan((FinalAngle-InitAngle)/2.0));   //get the center point.  derived using basic trig
	double Radius = getDistance(A, Center); //this may or may not work

	PathSegment P;
	P.seg_type = CURVE;
	P.seg_number = SegNum;
	P.seg_length = FinalAngle-InitAngle;
		if(P.seg_length > 3.14159) {
		P.seg_length -= 2 * 3.14159;
	} else if(P.seg_length < -3.14159) {
		P.seg_length += 2 * 3.14159;
	}

	P.curvature = 1/STD_TURN_RAD;
	// I know that curvature should be 1/Radius, but that didn't work for some reason so it was overriden
	P.curvature *= (P.seg_length>0)?1:-1;
	P.seg_length = fabs(P.seg_length);

	geometry_msgs::Point gmPoint;
	gmPoint.x = Center.x;
	gmPoint.y = Center.y;
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
	P.accel_limit = MAX_LINEAR_ACC;
	return P;
}

//these two functions were used in the GetCurveAndLines routine.  defunct.
void MoveBack1(Point3f A, Point3f B, PathSegment* Segment) // moves the start and end points as special cases
{
	A = (A-B)*0.5; //just want the distance from A to the midpoint
	Segment->ref_point.x += A.x;
	Segment->ref_point.y += A.y;
	Segment->seg_length += getDistance(A);
}
void MoveBack2(Point3f A, Point3f B, PathSegment* Segment)
{
	Segment->seg_length += getDistance(B-A)*0.5;
}

// this was designed for use in the insertTurns function
void GetCurveAndLines( Point3f A, Point3f B, Point3f C, PathSegment* FirstLine, PathSegment* Curve, PathSegment* SecondLine, int* SegNum)
{
	//find the points needed to generate 2 lines with a curve in between
	double Theta = dotProduct(A-B, B-C)/(getDistance(A-B)*getDistance(B-C));  //implement the math that I did earlier
	Point3f D = (A+C)*0.5;
	Point3f Center = B+(D-B)*(1/getDistance(D-B))*(STD_TURN_RAD/acos(tan(Theta/2.0)));
	Point3f Bprime = A+dotProduct(Center-A,B-A)*(B-A)*(1/getDistance(B-A));

	Point3f Bdoubleprime = B+dotProduct(Center-C,B-C)*(B-C)*(1/getDistance(B-C)); //the equation in the pic ben sent me is wrong I think.  C should substitute for A, not B for A and C for B like I did.
	Point3f Midpoint1 = A+(A-B)*0.5;  //midpoints are used in a sec
	Point3f Midpoint2 = C-(C-B)*0.5;
	if (dotProduct(Midpoint1, B-A) < dotProduct(Bprime, B-A) || dotProduct(Midpoint2, B-C) < dotProduct(Bdoubleprime, B-C))
	{
		(*FirstLine) = MakeLine(Midpoint1, B, (*SegNum)++);
		(*SecondLine)  = MakeLine(B,Midpoint2, (*SegNum)+1);
		(*Curve) = MakeTurnInPlace(tf::getYaw(FirstLine->init_tan_angle), tf::getYaw(SecondLine->init_tan_angle), convertGeoPointToPoint3f(SecondLine->ref_point), (*SegNum)++) ;
		(*SegNum)++;
	}
	else 
	{
		(*FirstLine) = MakeLine(Midpoint1, Bprime, (*SegNum)++);
		(*SecondLine) = MakeLine(Bdoubleprime, Midpoint2,(*SegNum)+1);
		(*Curve) = MakeCurve(tf::getYaw(FirstLine->init_tan_angle), tf::getYaw(SecondLine->init_tan_angle), convertGeoPointToPoint3f(FirstLine->ref_point), convertGeoPointToPoint3f(SecondLine->ref_point), (*SegNum)++) ;
		(*SegNum)++;
	}
}

// this was supposed to take a list of points and turn them into a series of lines and turns, following 
// the pattern (line, turn, line, line, turn, line ...) note that any two adjacent lines 
// are parallel and connect at start and end points to essentially make a single larger line segment.  

// this function was merged with parts of the bug algorithm because we only needed the special 
// cases of 90 and 45 degree turns. in the future, this will be included because it is much more 
// generically written
PathList insertTurns(list<Point2d> P)
{
	//convert from the list<Point2d> to an array of Point3's
	Point3f* PointList = (Point3f*)calloc(sizeof(Point3f),P.size());  //this should be the list of points that ben's algorithm puts out
	int PointListLength = P.size();
	
	list<Point2d>::iterator it; 
	int i = 0;
	for (it = P.begin(); it!=P.end(); it++)
	{
		PointList[i].x = it->x;
		PointList[i].y = it->y;
		PointList[i].z = 0;
		i++;
	}
	
	PathList ReturnVal;//the path list that we will eventually return
	vector<PathSegment> path = vector<PathSegment>(3*(PointListLength-2));//3(n-2) segments are needed when using the line turn line line turn line pattern
	ReturnVal.path_list.assign(path.begin(), path.end());
	//(PathSegment*)malloc(sizeof(PathSegment)*(3*(PointListLength))); //the equation for this comes from the path planner splitting each segment except for the first and last.
	int SegNum = 0;
	Point3f A, B, C; //points A,B,C for the line turn line pattern
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
Point3f findPointAlongCircle(Point3f startPoint, double initial_heading, double change_in_heading, double radius) {
		double heading = initial_heading;
		if(change_in_heading > 0) {
			//we are going positive angle, add 90
			heading += 3.14159 / 2;
		} else {
			heading -= 3.14159/2;
		}
		Point3f center = Point3f(startPoint.x + cos(heading)*radius, startPoint.y + sin(heading)*radius,0.0);
		//now invert heading and adjust it by by change_in_heading
		heading = heading - 3.14159 + change_in_heading;
		return Point3f(center.x + cos(heading)*radius, center.y + sin(heading)*radius, 0.0);
}

// Takes the list of points and joins them into lines with orientation, etc
// Assumes that the points are in order (this should be done in camera)
PathList joinPoints(sensor_msgs::PointCloud pointList)
{
    vector<PathSegment> lines;
    
    // This is the stupid way - no smooth rotations
    for (int i=0; i<pointList.points.size()-2; i++)
    {
        Point3f A = convertGeoPointToPoint3f(pointList.points[i]);
        Point3f B = convertGeoPointToPoint3f(pointList.points[i+1]);
        lines.push_back(MakeLine(A, B, i));
    }
    
    PathList pathList;
    pathList.path_list = lines;
    return pathList;
}

// Begin main loop, callbacks, etc

cv::Mat_<bool> *lastCSpace_Map;
cv::Mat_<bool> *lastVISION_Map;
cv::Mat_<bool> *lastSONAR_Map;

geometry_msgs::PoseStamped poseDes;
geometry_msgs::Pose goalPose; 
geometry_msgs::Pose mapOrigin;
geometry_msgs::PoseStamped poseActual;
sensor_msgs::PointCloud pointList;

bool LIDARcalled = false;
bool poseDescalled = false;
bool goalPosecalled = false;
bool poseActualcalled = false;
bool pointListcalled = false;

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

void pointList_Callback(const sensor_msgs::PointCloud::ConstPtr& newPointList)
{
    pointList = *newPointList;
    pointListcalled = true;
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
	ros::Subscriber sub6 = n.subscribe<sensor_msgs::PointCloud>("Cam_Cloud", 10, pointList_Callback);
	
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
		ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely
		ros::Time current_time = ros::Time::now();
		elapsed_time= ros::Time::now()-birthday;

		if(LIDARcalled && goalPosecalled) {
			if(!poseDescalled) {
				//this means we haven't used yet, so use our actual pose
				poseDes.pose.position.x = 7.57;
				poseDes.pose.position.y = 14.26;
				poseDes.pose.orientation =  tf::createQuaternionMsgFromYaw(-2.361);
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
			
			//PathList turns = bugAlgorithm(lastCSpace_Map, Point2d(goalPose.position.x, goalPose.position.y),poseDes, mapOrigin);
			PathList turns = joinPoints(pointList);
			
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
