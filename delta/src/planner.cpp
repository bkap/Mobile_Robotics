#include "command_publisher.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <list>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <eecs376_msgs/PathSegment.h>
#include <eecs376_msgs/PathList.h>
#include <eecs376_msgs/CrawlerDesiredState.h>
#include <visualization_msgs/Marker.h>
#include "CSpaceFuncs.h"

#define REFRESH_RATE 10

//define the 3 types of segments, these need to be consistent with dpcrawler
#define LINE 1
#define CURVE 2
#define POINT_TURN 3
#define STD_TURN_RAD 6.0//.6 //given in the assignment but changed because it seemed to work better

//define maximum speeds and accelerations
#define MAX_LINEAR .3
#define MAX_ANGULAR .5
#define MAX_LINEAR_ACC .5
#define MAX_ANGULAR_ACC .5

#define PI 3.141592

//uses some namespaces
using namespace cv;
using namespace std;
using namespace eecs376_msgs;
tf::TransformListener *tfl;

int segnum = 0;
// Point type conversions
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

// 'Distance' of one point = magnitude of A = distance from origin to A
float getDistance(Point3f A)
{
    return sqrt( A.x*A.x + A.y*A.y + A.z*A.z );
}

// used in GetCurveAndLines
float dotProduct(Point3f A, Point3f B)
{
    return A.x*B.x + A.y*B.y + A.z*B.z;
}

// Make a line from A to B
PathSegment MakeLine(Point3f A, Point3f B, int SegNum)
{
	PathSegment P;
	P.seg_type = LINE;
	P.seg_number = SegNum;
	P.seg_length = getDistance(A, B);
	P.ref_point = convertPoint3fToGeoPoint(A);
	Point3f vec = B-A;
	P.init_tan_angle = tf::createQuaternionMsgFromYaw(atan2(vec.y, vec.x));
	P.curvature = 1337;
	
	P.max_speeds.linear.x = MAX_LINEAR;
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

// Generate a turn in place given an initial heading, a final heading, and a point to turn on.
PathSegment MakeTurnInPlace (double InitAngle, double FinalAngle, Point3f ref_point, int SegNum)
{
	double amountToTurn = FinalAngle - InitAngle;
	if(amountToTurn > CV_PI) { amountToTurn -= 2 * CV_PI; }
	else if(amountToTurn < -CV_PI) { amountToTurn += 2 * CV_PI;}
	PathSegment P;
	P.seg_type = POINT_TURN;
	P.seg_number = SegNum;
	P.seg_length = fabs(amountToTurn); //how much to turn
	P.ref_point = convertPoint3fToGeoPoint(ref_point); //where to turn
	P.init_tan_angle = tf::createQuaternionMsgFromYaw(InitAngle); //initial heading as a quaternion
	
	P.curvature = (amountToTurn > 0)?1:-1; //direction to turn
	
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

// Make a curve given initial and final headings as well as start and end points
PathSegment MakeCurve(double InitAngle, double FinalAngle, Point3f A, Point3f B, int SegNum)
{
	Point3f M = (A+B)*(0.5);  //midpoint
	Point3f MA = M-A;  //vector from midpoint to A
	Point3f Center = M - Point3f(MA.y, -MA.y, 0)*(1.0/tan((FinalAngle-InitAngle)/2.0)); //get the center point.  derived using basic trig
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

// get the heading based on two points
double getHeading(Point3f a, Point3f b)
{
    return atan((b.x-a.x)/(b.y-a.y));
}

// Take three points, return a smooth path with an arc by the middle one
// replacing shady vector math of GetCurveAndLines with slightly less shady trigonometry
void GetCurveAndLines2(Point3f a, Point3f b, Point3f c, PathSegment* line1, PathSegment* curve, PathSegment* line2, int* segNum)
{
    cout << "\nshady trig getcurveandlines2\n";
    cout <<"a="<<a.x<<","<<a.y<<", b="<<b.x<<","<<b.y<<", c="<<c.x<<","<<c.y<<"\n";
    
    double radius = STD_TURN_RAD; // eh
    double psi1 = getHeading(a,b);
    double psi2 = getHeading(b,c);
    double theta1 = psi1 + PI/2.0; // theta is relative to center of circle
    double theta2 = psi2 + PI/2.0; // this could be wrong dpdg on curvature check it
        
    if (a.x < b.x) // turn left, this is probably completely wrong
    {
        cout << "turning left lolz\n";
        theta1 = psi1 - PI/2.0;
        theta2 = psi2 - PI/2.0;
    }
    // else turn right
    
    /*
    // from dpcrawler
    if (desState.des_rho >= 0) {
        theta = psiDes - PI/2;
    } else { // right hand
        theta = psiDes + PI/2;
    }*/
    double theta = theta2-theta1;
    double cornerDist = fabs(radius * sin(theta/2) / sin(PI/2 - theta/2));
    
    cout <<"psi1="<<psi1<<", psi2="<<psi2<<"\ntheta1="<<theta1<<", theta2="<<theta2<<", theta="<<theta<<"\ncornerdist="<<cornerDist<<"\n";
    
    Point3f b1 = Point3f(b.x - cornerDist*sin(psi1), b.y - cornerDist*cos(psi1), 0); // end point of first line
    Point3f b2 = Point3f(b.x + cornerDist*sin(psi2), b.y + cornerDist*cos(psi2), 0); // start point of second line
    
    cout <<"b1="<<b1.x<<","<<b1.y<<"\nb2="<<b2.x<<","<<b2.y<<"\n";
    
    *line1 = MakeLine(a, b1, (*segNum)++);
    *curve = MakeCurve(theta1, theta2, b1, b2, (*segNum)++);
    *line2 = MakeLine(b2, c, (*segNum)++);
}

/*
// this was designed for use in the insertTurns function
// uses shady vectors and appears to be wrongish
void GetCurveAndLines( Point3f A, Point3f B, Point3f C, PathSegment* FirstLine, PathSegment* Curve, PathSegment* SecondLine, int* SegNum)
{
    cout << "shady vector getcurveandlines\n";
	//find the points needed to generate 2 lines with a curve in between
	double Theta = acos(dotProduct(A-B, B-C)/(getDistance(A-B)*getDistance(B-C)));  //implement the math that I did earlier
	Point3f D = (A+C)*0.5;
	Point3f Center = B+(D-B)*(1/getDistance(D-B))*(STD_TURN_RAD/acos(tan(Theta/2.0))); //???
	Point3f Bprime = A+dotProduct(Center-A,B-A)*(B-A)*(1/getDistance(B-A));

	Point3f Bdoubleprime = B+dotProduct(Center-C,B-C)*(B-C)*(1/getDistance(B-C));
	//the equation in the pic ben sent me is wrong I think.  C should substitute for A, not B for A and C for B like I did.
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
*/

vector<PathSegment> oldPath;
int getFirstNotTooClose(int segnum, Point3f* PointList, int size) {
	Point3f p = convertGeoPointToPoint3f(oldPath[segnum].ref_point);
	double tan_angle = tf::getYaw(oldPath[segnum].init_tan_angle);
	p.x += oldPath[segnum].seg_length * cos(tan_angle);
	p.y += oldPath[segnum].seg_length * sin(tan_angle);
    for(int i = 0; i < size; i++) {
		//threshold = 0.5 meters
		if(getDistance(p, PointList[i]) > 0.5) {
			return i;
		}
	}
	return -1;
}

bool FirstTime = true;
// this was supposed to take a list of points and turn them into a series of lines and turns, following 
// the pattern (line, turn, line, line, turn, line ...)
PathList insertTurns(double initial_heading,sensor_msgs::PointCloud pointCloud)
{
    int pointListSize = pointCloud.points.size();
	Point3f* PointList = (Point3f*)calloc(sizeof(Point3f),pointCloud.points.size());
	// convert pointCloud to PointList
	for (int i=0; i<pointListSize; i++) {
	    PointList[i] = convertGeoPointToPoint3f(pointCloud.points[i]);
	}
PathList ReturnVal; //the path list that we will eventually return
vector<PathSegment> path; 
if(FirstTime)
{
	//FirstTime = false;
	//init_angle, final_angle, repoint, segnum
	double old_heading = initial_heading;
	path = vector<PathSegment>(pointListSize * 2);
	for(int i =0; i<pointListSize-1; i++)
	{
		double new_heading = atan2(PointList[i+1].y - PointList[i].y,PointList[i+1].x-PointList[i].x);
	 	path[2*i] = MakeTurnInPlace(old_heading, new_heading, PointList[i], 2*i);
	     path[2*i+1] = MakeLine(PointList[i], PointList[i+1], 2*i+1);
		old_heading = new_heading;
	}
	path.pop_back();
	oldPath = path;
	ReturnVal.path_list.assign(path.begin(), path.end());
}
else
{
	//get the first point on the suggested path list which is not too close to the current pose
	//TODO make this function and get the current segment by subscribing to it
	int StartIndex = getFirstNotTooClose(segnum, PointList, pointCloud.points.size());
	if(StartIndex != -1) {
		
	
		path = vector<PathSegment>(pointListSize-StartIndex+segnum+1); //the +1 is because segNum's start at 0
		for(int i = 0; i<segnum+1; i++)
		{
			path[i] = oldPath[i];
		}
		for(int i = 0; i < pointListSize - StartIndex; ++i) {
	
		     path[i+segnum + 1] = MakeLine(PointList[i], PointList[i+1], i+segnum+1);
		}
	} else {
		path = oldPath;
	}
	path.pop_back();
	oldPath = path;
	ReturnVal.path_list.assign(path.begin(), path.end());
}
    free(PointList);	
	return ReturnVal;//return the pathlist
}
//finds a point along a curve given initial parameters
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
PathList joinPoints(double initial_heading,sensor_msgs::PointCloud pointList)
{
    vector<PathSegment> lines;
    
    // This is the stupid way - no smooth rotations
    /*
    for (int i=0; i<pointList.points.size()-2; i++)
    {
        Point3f A = convertGeoPointToPoint3f(pointList.points[i]);
        Point3f B = convertGeoPointToPoint3f(pointList.points[i+1]);
        lines.push_back(MakeLine(A, B, i));
    }
    */
    
    // with smoothing, oh hey, this was already written
    PathList pathList = insertTurns(initial_heading,pointList);
    // write mah own
    //pathList = smoothLine(pointList);
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


bool LIDARcalled = true;
bool poseDescalled = false;
bool goalPosecalled = false;
bool poseActualcalled = false;
bool pointListcalled = true;

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
void segnum_Callback(const eecs376_msgs::CrawlerDesiredState::ConstPtr& crawledState) {
	segnum = (*crawledState).seg_number+1;
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
    //pointList = *newPointList;
    pointListcalled = true;
}

int main(int argc,char **argv)
{
	cout<<argc<<"=argc, argv's=\n";
	for (int i=0;i<argc;i++)
	cout <<i<<": "<< *argv[i] << "\n";

	ros::init(argc,argv,"pathPlanner");//name of this node
	tfl = new tf::TransformListener();
	double amount_to_change = 0.0;    
	cout<<"3\n";
	ros::NodeHandle n;
	double intial_heading;
	ros::Subscriber sub1 = n.subscribe<nav_msgs::OccupancyGrid>("CSpace_Map", 10, LIDAR_Callback);
	ros::Subscriber sub4 = n.subscribe<geometry_msgs::PoseStamped>("poseActual", 10, poseActual_Callback);
	ros::Subscriber sub5 = n.subscribe<geometry_msgs::Pose>("goalPose", 10, goalPose_Callback);
	ros::Subscriber sub6 = n.subscribe<sensor_msgs::PointCloud>("Cam_Cloud", 10, pointList_Callback);
	ros::Subscriber sub2 = n.subscribe<eecs376_msgs::CrawlerDesiredState>("crawlerDesState",1,segnum_Callback);

	ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );

	// hax to test
	while(!poseActualcalled) {ros::spinOnce();}

	// let's make some fake points for a path WOO
	sensor_msgs::PointCloud pointList;
	geometry_msgs::Point32 origin;
	origin.x = poseActual.pose.position.x;
	origin.y = poseActual.pose.position.y;
	pointList.points.push_back(origin);
	geometry_msgs::Point32 p;
		//along front
		//p.x = 4.326;
		//p.y = 4.26;
		/*//inside
		p.x = 5.2;
		p.y = 12.15; */
		//along back
		p.x = -67.4;
		p.y = -20.2;
		pointList.points.push_back(p);
	    
        geometry_msgs::Point32 p2;
		//p2.x = 7.848;
		//p2.y = 9.336;
		//p2.x = -3.15;
		//p2.y = 20.4;
		p2.x = -63.8;
		p2.y = -19.4;
		pointList.points.push_back(p2);
		
        geometry_msgs::Point32 p3;
		//p3.x = 16.227;
		//p3.y = 18.893;
		//p3.x = -0.75;
		//p3.y = 23.05;
		p3.x = -59.8;
		p3.y = -16.6;
		pointList.points.push_back(p3);
		double initial_heading= tf::getYaw(poseActual.pose.orientation);
	    PathList turns = joinPoints(initial_heading,pointList);
	    
	    // and print them out
	    ros::Publisher path_pub = n.advertise<eecs376_msgs::PathList>("pathList",10);
	
	    cout<<"3\n";
	    ros::Duration elapsed_time; // define a variable to hold elapsed time
	    ros::Rate naptime(REFRESH_RATE); //will perform sleeps to enforce loop rate of "10" Hz
	    while (ros::ok()&&!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly. Wait until ros::Time::now() will be valid, but let any callbacks happen
          	cout<<"3\n";
	    ros::Time birthday = ros::Time::now();
	    //desired_pose.header.stamp = birthday;
	   // while (ros::ok()&&!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce(); // wait until there is transform data available before starting our controller loopros::Time birthday= ros::Time::now(); // get the current time, which defines our start time, called "birthday"
	    cout<<"3\n";
	    ROS_INFO("birthday started as %f", birthday.toSec());
      
	    while (ros::ok()) // do work here
	    {
		    ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely
		    ros::Time current_time = ros::Time::now();
		    elapsed_time= ros::Time::now()-birthday;

		    if(LIDARcalled && goalPosecalled && pointListcalled) {
			    if(!poseDescalled) {
				    //this means we haven't used yet, so use our actual pose
				    poseDes.pose.position.x = 7.57;
				    poseDes.pose.position.y = 14.26;
				    poseDes.pose.orientation =  tf::createQuaternionMsgFromYaw(-2.361);
			    }
		
			    list<geometry_msgs::Point> points;
			    geometry_msgs::Point p;
			    /*for(int i=0;i<lastCSpace_Map->size().height; i++)
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
			*/
			    //PathList turns = bugAlgorithm(lastCSpace_Map, Point2d(goalPose.position.x, goalPose.position.y),poseDes, mapOrigin);
			    PathList turns = joinPoints(initial_heading,pointList);
			
			
			    cout<<"publishing\n";
			    path_pub.publish(turns);
			    cout<<"3published"<<"\n";	
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
