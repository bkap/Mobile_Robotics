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
#include<geometry_msgs/Pose.h> //data type for Pose combined with frame and timestamp
#include<tf/transform_datatypes.h> // for tf::getYaw
#include<tf/transform_listener.h> // for the TransformListener class that abstracts away a lot of tf
#include <eecs376_msgs/PathSegment.h>
#include <eecs376_msgs/PathList.h>
#include <eecs376_msgs/CrawlerDesiredState.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace cv;
using namespace eecs376_msgs;
using namespace std;
const double REFRESH_RATE = 10;
const double pi = 3.141592;

PathList pathlist;
CrawlerDesiredState curState;
cv::Mat_<bool> *lidarMap;
geometry_msgs::Pose mapOrigin;

bool crawlerDesStateCalled = false;
void crawlerDesStateCallback(const CrawlerDesiredState::ConstPtr& desState)
{
    curState = *desState;
	crawlerDesStateCalled = true;
}
bool pathListCalled = false;
void pathListCallback(const PathList::ConstPtr& paths)
{
    // get the path list
	pathListCalled = true;
    pathlist = *paths;
}

//http://www.ros.org/doc/api/nav_msgs/html/msg/OccupancyGrid.html

bool lidarMapCalled = false;
void lidarMapCallback(const boost::shared_ptr<nav_msgs::OccupancyGrid  const>& newLidarMap)
{
    // get the map, as a matrix (1 = occupied, 0 = empty; 5 cm grid)
    lidarMap = getMap(*newLidarMap);
    mapOrigin = (*newLidarMap).info.origin;
	lidarMapCalled = true;
}

// check along the path within the specified distance along the specified path
// and test cspace map pixels for occupancy
bool clearPath(double brakingDist)
{
    double dist = 0.0;
    int curSeg = curState.seg_number;
    double psiCur = tf::getYaw(curState.des_pose.orientation);
    
    int curX = (curState.des_pose.position.x - mapOrigin.position.x)/CSPACE_RESOLUTION;
    int curY = (curState.des_pose.position.y - mapOrigin.position.y)/CSPACE_RESOLUTION;
    
    while (dist < brakingDist) {    // move forward one block in the grid
        dist += CSPACE_RESOLUTION;
        
        // this is basically dpcrawler again
        switch (pathlist.path_list[curSeg].seg_type) {
            case 1: // line
                curX += CSPACE_RESOLUTION * cos(psiCur);
                curY += CSPACE_RESOLUTION * sin(psiCur);
                break;
            case 2: // arc: speedNominal is the tangential velocity (v = w R)
            {   // lsegdes is distance s traveled along the arc: s = R * theta
                double theta = 0.0; // angle from center of curvature
                double rho = pathlist.path_list[curSeg].curvature;
                
                if (psiCur >= 0)
                    theta = psiCur - pi/2;
                else
                    theta = psiCur + pi/2;
                
                theta += CSPACE_RESOLUTION * fabs(rho);
                psiCur += CSPACE_RESOLUTION * fabs(rho);
                
                curX += cos(theta) / rho;
                curY += sin(theta) / rho;
                break;
            }
            case 3: // rotate about point
                // "distance" is actually the angle rotated, x and y do not change
                break;
        }
        
        if (dist >= fabs(pathlist.path_list[curSeg].seg_length)) { // end of path segment, update
            brakingDist -= dist;//fabs(pathlist.path_list[curSeg].seg_length);
            dist = 0.0;
            curSeg++;
        }
        
        // check map
        if ((*lidarMap)(curX,curY) == 1)  {// occupied
            cout << "pro: clearPath OCCUPIED\n";
            return false;
        }
    }
    cout << "pro: clearPath CLEAR\n";
    return true;
}

// Calculate how far robot will go, with max deceleration, to get to the goal speed
double distanceToGoalSpeed(double goal_speed)
{
    // Currently only have one acceleration limit, so these are the same
    double aLinMax = pathlist.path_list[curState.seg_number].accel_limit;
    double aAngMax = pathlist.path_list[curState.seg_number].accel_limit;
    
    double brakingDistance = 0.0;
    switch (curState.seg_type) {
        case 1: // line
        case 2: // arc
            brakingDistance = curState.des_speed * (curState.des_speed - goal_speed) / aLinMax - pow(curState.des_speed - goal_speed, 2) / (2 * aLinMax);
            break;
        case 3: // point
            brakingDistance = curState.des_speed * (curState.des_speed - goal_speed) / aAngMax - pow(curState.des_speed - goal_speed, 2) / (2 * aAngMax);
            break;
    }
    cout << "distToGoalSpeed " << goal_speed << " is " << brakingDistance << "\n";
    return brakingDistance;
}

// Calculate the remaining distance to the end of the path segment
double distanceRemaining()
{
    double xCur = curState.des_pose.position.x;
    double yCur = curState.des_pose.position.y;
    double psiCur = tf::getYaw(curState.des_pose.orientation);
    
    unsigned int nextSeg = curState.seg_number + 1;
    double xDes = 0.0;
    double yDes = 0.0;
    double psiDes = 0.0;
    if (nextSeg == pathlist.path_list.size()) // at end of path
    {
        nextSeg = curState.seg_number;
        
        // find endpoint coordinates... x and y are only relevant for lines, so, only worry about those
        xDes = pathlist.path_list[nextSeg].ref_point.x + cos(tf::getYaw(pathlist.path_list[nextSeg].init_tan_angle)) * pathlist.path_list[nextSeg].seg_length;
        yDes = pathlist.path_list[nextSeg].ref_point.x + sin(tf::getYaw(pathlist.path_list[nextSeg].init_tan_angle)) * pathlist.path_list[nextSeg].seg_length;
        
        // find psiDes
        if (curState.seg_type == 2) { //arc: psi = psi0 + s/rho
            psiDes = tf::getYaw(pathlist.path_list[nextSeg].init_tan_angle) + pathlist.path_list[nextSeg].seg_length / pathlist.path_list[nextSeg].curvature;
        } else if (curState.seg_type == 3) { // point: psi = psi0 + dpsi
            psiDes = tf::getYaw(pathlist.path_list[nextSeg].init_tan_angle) + pathlist.path_list[nextSeg].seg_length;
        }
    } else {
        xDes = pathlist.path_list[nextSeg].ref_point.x;
        yDes = pathlist.path_list[nextSeg].ref_point.y;
        psiDes = tf::getYaw(pathlist.path_list[nextSeg].init_tan_angle);
    }
    cout << "pro: return distRem\n";
    switch (curState.seg_type) {
        case 1: // distance along line
            return pow(pow(xDes-xCur, 2) + pow(yDes-yCur, 2), 0.5);
        case 2: // distance along arc
            return (psiDes - psiCur) / curState.des_rho;
        case 3: // angle remaining
            return (psiDes - psiCur);

	assert(1337==0);//control shouldn't get here.
	return 9001; //it's over 9000
    }
}

/*
Subscribes: CrawlerDesiredState, pathList, CspaceMap (s)

Behavior: start at current point on pathlist
if path is clear:
    if room, speed up
    if within stopping distance, slow down to goal speed
if obstruction:
    brake

Publishes: eecs376_msgs/CrawlerDesiredState, but with the des_speed field filled in with a correct value
*/
int main(int argc,char **argv)
{   
    ros::init(argc,argv,"profiler");//name of this node
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<CrawlerDesiredState>("NominalSpeed",1);
    
    // list of subscribers
	ros::Subscriber subCrawlerDesState = n.subscribe<CrawlerDesiredState>("crawlerDesState", 1, crawlerDesStateCallback);
	ros::Subscriber subPathList = n.subscribe<PathList>("pathList", 1, pathListCallback);
	ros::Subscriber subLidar = n.subscribe<nav_msgs::OccupancyGrid>("LIDAR_Map", 1, lidarMapCallback);
	
	ros::Rate naptime(REFRESH_RATE); //will perform sleeps to enforce loop rate of 10 Hz
    while (!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly.
    //Wait until ros::Time::now() will be valid, but let any callbacks happen
    
    while (ros::ok()) // do work here
    {
        ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely
        
        // Calculate braking and slowing distances
        double vMax, aMax, vGoal;
        
        // ramp to zero if you're on the last path segment
        bool end_of_path = false;
        cout << "\nPROFILER\n";
        if(crawlerDesStateCalled && pathListCalled && lidarMapCalled) {
			if (curState.seg_number == pathlist.path_list.size() - 1) {
                cout << "\npro: End of path\n";
            	end_of_path = true;
        	}
        	vGoal = 0.0;
 
        	if (curState.seg_type == 3) { // point
        	    vMax = pathlist.path_list[curState.seg_number].max_speeds.angular.z;
            	aMax = pathlist.path_list[curState.seg_number].accel_limit;
            	if (!end_of_path)
                	vGoal = pathlist.path_list[curState.seg_number + 1].max_speeds.angular.z;
        	} else { // line or arc
        	    vMax = pathlist.path_list[curState.seg_number].max_speeds.linear.x;
            	aMax = pathlist.path_list[curState.seg_number].accel_limit;
            	if (!end_of_path)
                	vGoal = pathlist.path_list[curState.seg_number + 1].max_speeds.linear.x;
        	}
            
        	double brakingDistance = distanceToGoalSpeed(0.0);
        	double slowingDistance = distanceToGoalSpeed(vMax);
        	double distToGo = distanceRemaining();
        	
        	cout << "pro: brakingdist=" << brakingDistance << ", slowingDist=" << slowingDistance << ", distToGo=" << distToGo << "\n";
 
        	// Ramp velocity, set des_speed
        	if (clearPath(brakingDistance)) {
        	    cout << "\npro: clearpath woo";
            	if (slowingDistance < distToGo) {   // go to maximum velocity
                    cout << " TO THE MAX which is " << vGoal << " vs. " << curState.des_speed << " + " << aMax << " / " << REFRESH_RATE;
                	curState.des_speed = min(curState.des_speed + aMax / REFRESH_RATE, vGoal);
            	} else {    // ramp speed down to goal speed
            	    cout << " goal speed is slowsauce " << vGoal;
                	curState.des_speed = max(curState.des_speed - aMax / REFRESH_RATE, vGoal);
            	}
        	} else {
            	// go to zero
            	cout << " BRAKINGGGGG";
            	curState.des_speed = max(curState.des_speed - aMax / REFRESH_RATE, 0.0);
        	}
            cout << "\npro: des_speed = " << curState.des_speed;
        	pub.publish(curState); // publish the CrawlerDesiredState
	    }
	    naptime.sleep(); // enforce desired update rate
    }
    return 0;   // this code will only get here if this node was told to shut down, which is
                // reflected in ros::ok() is false
}
