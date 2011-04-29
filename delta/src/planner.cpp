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
#include <visualization_msgs/MarkerArray.h>
#include "CSpaceFuncs.h"
#include "plannerFuncs.cpp"

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
int goalnum = 0;
geometry_msgs::Pose des_pose;
double distanceOnSeg = 0.0;

cv::Mat_<bool> *lastCSpace_Map;
cv::Mat_<bool> *lastVISION_Map;
cv::Mat_<bool> *lastSONAR_Map;
cv::Mat_<char> lastCSpace_CharMap = Mat::zeros(900,900,CV_8S); // ... shady pointer or is this ok?

geometry_msgs::PoseStamped poseDes;
geometry_msgs::Pose goalPose; 
geometry_msgs::Pose mapOrigin;
double mapResolution;
geometry_msgs::PoseStamped poseActual;
sensor_msgs::PointCloud pointList;


bool LIDARcalled = false;
bool poseDescalled = false;
bool goalPosecalled = false;
bool poseActualcalled = false;
bool pointListcalled = true;

Point3f convertGridToMapCoords(Point2i grid) {
	Point3f mapcoords;

	cout<<"Map Resolution "<<mapResolution<<"\n";
	cout<<"Origin "<<mapOrigin.position.x<<","<<mapOrigin.position.y<<"\n";
	mapcoords.x = grid.x * mapResolution + mapOrigin.position.x;
	mapcoords.y = grid.y * mapResolution + mapOrigin.position.y;
	mapcoords.z = 0;
	return mapcoords;
}
Point2i convertMapToGridCoords(Point3f map) {

	cout<<"Map Resolution "<<mapResolution<<"\n";
	cout<<"Origin "<<mapOrigin.position.x<<","<<mapOrigin.position.y<<"\n";
	Point2i gridcoords;
	gridcoords.x = (int)((map.x - mapOrigin.position.x) / mapResolution);
	gridcoords.y = (int)((map.y - mapOrigin.position.y) / mapResolution);
	return gridcoords;
}
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

Point2i convertGeoPointToPoint2i(geometry_msgs::Point32 geopt)
{
	
	return convertMapToGridCoords(convertGeoPointToPoint3f(geopt));
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
	//double Radius = getDistance(A, Center); //this may or may not work

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
PathList insertTurns(double initial_heading,vector<Point3f> points, int initialSegNum)
{
PathList ReturnVal; //the path list that we will eventually return
vector<PathSegment> path; 
	//init_angle, final_angle, repoint, segnum
	double old_heading = initial_heading;
	path = vector<PathSegment>(points.size() * 2);
	for(int i =0; i<points.size()-1; i++)
	{
		double new_heading = atan2(points[i+1].y - points[i].y, points[i+1].x-points[i].x);
		if(fabs(old_heading - new_heading) < 0.2) {
	 		path.push_back(MakeTurnInPlace(old_heading, new_heading, points[i], path.size()+ initialSegNum));
		}
	     path.push_back(MakeLine(points[i], points[i+1], path.size() + initialSegNum));
		old_heading = new_heading;
	}
	ReturnVal.path_list.assign(path.begin(), path.end());
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


PathList *prevList;
vector<int> goalSegnums;
// Call A* search for path, lots of conversions needed so put in separate function
// pointList is the list of goal points (first point is assumed to be origin)
PathList callAStar(sensor_msgs::PointCloud pointList, double initial_heading)
{
	PathList turns;

    	// TODO: need a Mat not a Mat_<bool> as given by mapper ... mapper publishes an occupancyGrid which is converted into a Mat_<bool> in CSpaceFuncs
	cout<<"PLANNER: make char map\n";
    	Mat_<char> mapChar = lastCSpace_CharMap;
    	//lastCSpace_Map.convertTo(mapChar, CV_8SC1); //nope.
    	//find starting point
		Point3f startPoint;
		double heading = initial_heading;
		int startLoop = 1;
		cout<<"PLANNER:if condition that checks for null\n";
		if(prevList != NULL) {
			if(segnum > goalSegnums[goalnum ]) {
				goalnum++;
			}
			startLoop = goalnum + 1;
			for(int i = 0; i < segnum; i++) {
				turns.path_list.push_back(prevList->path_list[i]);
			}
			//TODO: might have an off-by-one here
			PathSegment oldSeg = oldPath[segnum];
			//put all segments up to segnum into turns

		cout<<"PLANNER:start pos\n";
			startPoint = convertGeoPointToPoint3f(des_pose.position);

			double heading = tf::getYaw(des_pose.orientation);
			if(oldSeg.seg_type == 1) {
				//it's a line
				//probably shouldn't hardcode acceleration, but we have to
				if(oldSeg.seg_length - distanceOnSeg > (oldSeg.max_speeds.linear.x * oldSeg.max_speeds.linear.x / 0.1)) {
			
					startPoint.x += (oldSeg.max_speeds.linear.x * oldSeg.max_speeds.linear.x / 0.1) * cos(heading);
			
					startPoint.y += (oldSeg.max_speeds.linear.x * oldSeg.max_speeds.linear.x / 0.1) * sin(heading);
				//shrink the segment length
					oldPath[segnum].seg_length = distanceOnSeg + oldSeg.max_speeds.linear.x * oldSeg.max_speeds.linear.x / 0.1;
				} else {
					startPoint.x = oldSeg.ref_point.x + cos(heading) * oldSeg.seg_length;
					startPoint.y = oldSeg.ref_point.y + sin(heading) * oldSeg.seg_length;
				}
			} else if (oldSeg.seg_type == 3) {

			//turn in place, adjust heading
				heading += oldSeg.seg_length - distanceOnSeg;
			}
		} else {
			prevList = (PathList*)malloc(sizeof(PathList));
			startPoint = convertGeoPointToPoint3f(pointList.points[0]);
		}
    	//vector<Point2i> aStar (Mat map, Point2i start, Point2i end)
cout<<"PLANNER:calling a*\n";
		for(uint i = startLoop; i < pointList.points.size(); i++) {
		cout<<i<<"\n";
    		vector<Point2i> segPts = aStar(mapChar, convertMapToGridCoords(startPoint), convertGeoPointToPoint2i(pointList.points[i]));
			vector<Point3f> mapPts(segPts.size());
		cout<<"PLANNER:transform\n";

			for(int k = 0; k <(int)segPts.size(); k++) {
				mapPts[k] = convertGridToMapCoords(segPts[k]);

			}
			goalSegnums[i-1] = turns.path_list.size() + mapPts.size();
    		if(i + 1 < pointList.points.size()) {
			//get startPos for next iteration

			cout<<"almost done\n";

				startPoint = convertGeoPointToPoint3f(pointList.points[i]);
				 
			}
			

		// convert vector<Point2i> to vector<Point3f>
			PathList pathseg = insertTurns(initial_heading, mapPts, turns.path_list.size());
			for (uint j=0; j<pathseg.path_list.size(); j++)
				turns.path_list.push_back(pathseg.path_list[j]);

			PathSegment lastSeg = turns.path_list[turns.path_list.size() - 1];
			switch(lastSeg.seg_type) {
				case 1:
					heading = tf::getYaw(lastSeg.init_tan_angle);
					break;
				case 3:
					heading = tf::getYaw(lastSeg.init_tan_angle) + lastSeg.seg_length;
					if(heading > CV_PI) {
						heading -= 2 * CV_PI;
					} else if(heading < -CV_PI) {
						heading += 2 * CV_PI;
					}
					break;

			}

		}
			
	*prevList = turns;
    return turns;
}

// Begin main loop, callbacks, etc
nav_msgs::OccupancyGrid cspace;
void LIDAR_Callback(const boost::shared_ptr<nav_msgs::OccupancyGrid  const>& CSpace_Map)
{
	cspace = *CSpace_Map;
	//cout << "recieved width: " <<  (*CSpace_Map).info.width<< endl;
	//if(lastCSpace_Map != NULL) {
	//	delete lastCSpace_Map;
	//}
	cout<<"PLANNER: lidar 1\n";
	//lastCSpace_Map = getMap(*CSpace_Map); // stored as a Mat_<bool>
	cout<<"PLANNER: lidar 2\n";
	//lastCSpace_CharMap = cv::Mat((*CSpace_Map).data,true);

	Mat_<char> temp = Mat::zeros(ceil(cspace.info.width),ceil(cspace.info.width),CV_8S);
	ROS_INFO("temp created with size %d x %d",temp.rows,temp.cols);
	temp.data = (uchar*) &(CSpace_Map->data[0]);
	cout<<"WTFWTFWTF\n";
	for(int i = 0; i<(int)cspace.data.size(); i++)
	{
		lastCSpace_CharMap(i/lastCSpace_CharMap.cols, i%lastCSpace_CharMap.cols)=(char)cspace.data[i];
	}

	//temp.convertTo(lastCSpace_CharMap,CV_8S);
	//cvNamedWindow("last map",CV_WINDOW_AUTOSIZE);
	//imshow("last map",lastCSpace_CharMap);
	//waitKey(-1);
	//temp.copyTo(lastCSpace_CharMap);
	cout<<"PLANNER: lidar 3\n";
	//lastCSpace_CharMap = lastCSpace_CharMap.reshape(CSpace_Map->width);
	cout<<"PLANNER: lidar 4\n";
	mapOrigin = cspace.info.origin;
	mapResolution = cspace.info.resolution;
	LIDARcalled = true;
}
void segnum_Callback(const eecs376_msgs::CrawlerDesiredState::ConstPtr& crawledState) {
	segnum = (*crawledState).seg_number+1;
	distanceOnSeg = (*crawledState).des_lseg;
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
	ros::init(argc,argv,"pathPlanner");//name of this node

	tfl = new tf::TransformListener();
	//double amount_to_change = 0.0;    
	//double intial_heading;
	
	cout<<"PLANNER:Subscribing to stuff!\n";
	
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe<nav_msgs::OccupancyGrid>("CSpace_Map", 10, LIDAR_Callback);
	ros::Subscriber sub4 = n.subscribe<geometry_msgs::PoseStamped>("poseActual", 10, poseActual_Callback);
	ros::Subscriber sub5 = n.subscribe<geometry_msgs::Pose>("goalPose", 10, goalPose_Callback);
	ros::Subscriber sub6 = n.subscribe<sensor_msgs::PointCloud>("Cam_Cloud", 10, pointList_Callback);
	ros::Subscriber sub2 = n.subscribe<eecs376_msgs::CrawlerDesiredState>("crawlerDesState",1,segnum_Callback);
	// Stuff for path visualization

	cout<<"PLANNER:Publishing stuff\n";
	ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
	visualization_msgs::MarkerArray markers;
	
	// hax to test
	while(!poseActualcalled) {ros::spinOnce();}


	cout<<"PLANNER:WOOOO\n";
	// let's make some fake points for a path WOO
	sensor_msgs::PointCloud pointList;
	geometry_msgs::Point32 origin;
	origin.x = poseActual.pose.position.x;
	origin.y = poseActual.pose.position.y;
	pointList.points.push_back(origin);	// start points
	

	cout<<"PLANNER:making goal points\n";
	// goal points (these need changed for actual demo)
	geometry_msgs::Point32 p;
		
	//along front
	//p.x = 4.326;
	//p.y = 4.26;
	/*//inside
	p.x = 5.2;
	p.y = 12.15; */
	//along back
	p.x = 5.47;
	p.y = 12.24;
	pointList.points.push_back(p);
    
    geometry_msgs::Point32 p2;
	//p2.x = 7.848;
	//p2.y = 9.336;
	//p2.x = -3.15;
	//p2.y = 20.4;
	p2.x = -1.03;
	p2.y = 18.09;
	pointList.points.push_back(p2);
	
    geometry_msgs::Point32 p3;
	//p3.x = 16.227;
	//p3.y = 18.893;
	//p3.x = -0.75;
	//p3.y = 23.05;
	p3.x = .57;
	p3.y = 24.59;
	pointList.points.push_back(p3);
	double initial_heading= tf::getYaw(poseActual.pose.orientation);
	
	
//	PathList turns = callAStar(pointList, initial_heading);
	
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
		    //PathList turns = joinPoints(initial_heading,pointList);
			cout<<"PLANNER:calling A*\n";
			PathList turns = callAStar(pointList, initial_heading);
			cout<<"PLANNER:called A*\n";
			// Publish a visualization of the points
			//markers = visualizePoints(turns)
			vis_pub.publish(markers);
		
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
