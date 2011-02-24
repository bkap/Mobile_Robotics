#include<ros/ros.h>
#include<math.h>
#include<iostream>
#include<geometry_msgs/Pose.h> //data type for Pose combined with frame and timestamp
#include<tf/transform_datatypes.h> // for tf::getYaw
#include<tf/transform_listener.h> // for the TransformListener class that abstracts away a lot of tf
#include <eecs376_msgs/PathSegment.h>
#include <eecs376_msgs/PathList.h>
#include <eecs376_msgs/CrawlerDesiredState.h>
#include <nav_msgs/OccupancyGrid.h>
#include "command_publisher.h"

using namespace eecs376_msgs;
using namespace std;
const double REFRESH_RATE = 0.1;

PathList pathlist;
CrawlerDesiredState curState;
CSpaceMap cspaceMap;

void CrawlerDesiredStateCallback(const CrawlerDesiredState::ConstPtr& desState)
{
    curState = *desState;
}

void pathListCallback(const PathList::ConstPtr& paths)
{
    // get the path list
    pathlist = *paths;
}

//http://www.ros.org/doc/api/nav_msgs/html/msg/OccupancyGrid.html
void cspaceMapCallback(const nav_msgs::OccupancyMap::ConstPtr& csMap)
{
    // get the cspacemap
    cspaceMap = *csMap;
}

// TODO: this
bool clearPath(double dist)
{
    // check along the path within braking distances
    // and test cspace map pixels for occupancy
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
    return brakingDistance;
}

// Calculate the remaining distance to the end of the path segment
double distanceRemaining()
{
    double xCur = curState.des_pose.position.x;
    double yCur = curState.des_pose.position.y;
    double psiCur = tf::getYaw(curState.des_pose.orientation);
    double xDes = pathlist.path_list[curState.seg_number+1].ref_point.x;
    double yDes = pathlist.path_list[curState.seg_number+1].ref_point.y;
    double psiDes = tf::getYaw(pathlist.path_list[curState.seg_number+1].init_tan_angle);
    
    switch (curState.seg_type) {
        case 1: // distance along line
            return pow(pow(xDes-xCur, 2) + pow(yDes-yCur, 2), 0.5);
        case 2: // distance along arc
            return (psiDes - psiCur) / curState.des_rho;
        case 3: // angle remaining
            return (psiDes - psiCur);
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
    ros::Publisher pub = n.advertise<CrawlerDesiredState>("speedprofiler",1);
    
    // list of subscribers
    ros::Subscriber subCrawlerDesiredState = n.subscribe<CrawlerDesiredState>("CrawlerDesiredState", 1, CrawlerDesiredStateCallback);
	ros::Subscriber subPathList = n.subscribe<PathList>("pathList", 1, pathListCallback);
	ros::Subscriber subCspaceMap = n.subscribe<CSpaceMap>("cspaceMap", 1, cspaceMapCallback);
	
	ros::Rate naptime(1/REFRESH_RATE); //will perform sleeps to enforce loop rate of 10 Hz
    while (!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly.
    //Wait until ros::Time::now() will be valid, but let any callbacks happen
    
    while (ros::ok()) // do work here
    {
        ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely
        
        // Calculate braking and slowing distances
        double vMax, aMax, vGoal;
 
        if (curState.seg_type == 3) {
            vMax = pathlist.path_list[curState.seg_number].max_speeds.angular.z;
            aMax = pathlist.path_list[curState.seg_number].accel_limit;
            vGoal = pathlist.path_list[curState.seg_number + 1].max_speeds.angular.z;
        } else {
            vMax = pathlist.path_list[curState.seg_number].max_speeds.linear.x;
            aMax = pathlist.path_list[curState.seg_number].accel_limit;
            vGoal = pathlist.path_list[curState.seg_number + 1].max_speeds.linear.x;
        }
        
        double brakingDistance = distanceToGoalSpeed(0.0);
        double slowingDistance = distanceToGoalSpeed(vMax);
        double distToGo = distanceRemaining();
 
        // Ramp velocity, set des_speed
        if (clearPath(brakingDistance)) {
            if (slowingDistance < distToGo) {   // go to maximum velocity
                curState.des_speed = min(curState.des_speed + aMax * REFRESH_RATE, vGoal);
            } else {    // ramp speed down to goal speed
                curState.des_speed = max(curState.des_speed - aMax * REFRESH_RATE, 0.0);
            }
        } else {
            // go to zero
            curState.des_speed = max(curState.des_speed - aMax * REFRESH_RATE, 0.0);
        }

        pub.publish(curState); // publish the CrawlerDesiredState
	    
	    naptime.sleep(); // enforce desired update rate
    }
    return 0;   // this code will only get here if this node was told to shut down, which is
                // reflected in ros::ok() is false
}
