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

// TODO: get these from subscribed topics somehow
double lsegDes, xCur, yCur, psiCur, rhoCur, curSpeed, vLinMax, vAngMax, aLinMax, aAngMax;
int segType, segNumber;
Path[] pathlist;
CrawlerDesState curState;
CSpaceMap cspaceMap;
// path segment options
double pvLinMax, pvAngMax, paLinMax, paAngMax, xDes, yDes, rhoDes, psiInit;

// TODO: all of these signatures are wrong but I don't know what to put here
void crawlerDesStateCallback(const CrawlerDesiredState::ConstPtr& desState)
{
    curState = desState;
    // get the current stats
    lsegDes = 0.0;
    xCur = 0.0;
    yCur = 0.0;
    psiCur = 0.0;
    rhoCur = 0.0;
    segType = 0;
    segNumber = 0;
    curSpeed = 0.0;
}

void pathListCallback(const PathList::ConstPtr& paths)
{
    // get the path list
    pathlist = paths; // assigned here
    /*
    each path segment includes:
    segNum
    segLen
    segType
    referencePt
    initTanAng (initial heading)
    curvature (rho)
    vLinMax
    vAngMax
    aLinMax
    aAngMax
    */
}

// probably wrong
//http://www.ros.org/doc/api/nav_msgs/html/msg/OccupancyGrid.html
void cspaceMapCallback(const nav_msgs::OccupancyMap::ConstPtr& csMap)
{
    // get the cspacemap
    cspaceMap = csMap;
}

// TODO: this
bool clearPath(double dist_lin, double dist_ang)
{
    // check along the path within braking distances
    // and test cspace map pixels for occupancy
    return true;
}

// Calculate how far robot will go, with max deceleration, to get to the goal speed
double distanceToGoalSpeed(double goal_speed)
{
    double brakingDistance = 0.0;
    switch (segType) {
        case 1: // line
        case 2: // arc
            brakingDistance = curSpeed * (curSpeed - goal_speed) / aLinMax - pow(curSpeed - goal_speed, 2) / (2 * aLinMax);
            break;
        case 3: // point
            brakingDistance = curSpeed * (curSpeed - goal_speed) / aAngMax - pow(curSpeed - goal_speed, 2) / (2 * aAngMax);
            break;
    }
    return brakingDistance;
}

// Calculate the remaining distance to the end of the path segment
double distanceRemaining()
{
    switch (segType) {
        case 1: // distance along line
            return pow(pow(xDes-xCur, 2) + pow(yDes-yCur, 2), 0.5);
        case 2: // distance along arc
            return (psiDes - psiCur) / rhoDes;
        case 3: // angle remaining
            return (psiDes - psiCur);
    }
}

/*
Subscribes: CrawlerDesState, pathList, CspaceMap (s)

Behavior: start at current point on pathList
if path is clear:
    if room, speed up
    if within stopping distance, slow down
if obstruction:
    brake

Publishes: eecs376_msgs/CrawlerDesiredState, but with the des_speed field filled in with a correct value
*/
int main(int argc,char **argv)
{   
    ros::init(argc,argv,"profiler");//name of this node
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<CrawlerDesState>("speedprofiler",1);
    
    // list of subscribers
    // TODO: not sure if these are the right strings, also these are the wrong classes for pathlist and cspacemap
	ros::Subscriber subCrawlerDesState = n.subscribe<CrawlerDesState>("crawlerDesState", 1, crawlerDesStateCallback);
	ros::Subscriber subPathList = n.subscribe<PathList>("pathList", 1, pathListCallback);
	ros::Subscriber subCspaceMap = n.subscribe<CSpaceMap>("cspaceMap", 1, cspaceMapCallback);
	
	ros::Rate naptime(1/REFRESH_RATE); //will perform sleeps to enforce loop rate of 10 Hz
    while (!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly.
    //Wait until ros::Time::now() will be valid, but let any callbacks happen
    
    while (ros::ok()) // do work here
    {
        ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely
        
        // Calculate braking and slowing distances
        // TODO: get the goal velocities correctly
        double vMax, aMax, vGoal;
        if (segType == 3) {
            vMax = vAngMax;
            aMax = aAngMax;
            vGoal = pvAngMax;
        } else {
            vMax = vLinMax;
            aMax = aLinMax;
            vGoal = pvLinMax;
        }
        double brakingDistance = distanceToGoalSpeed(0.0);
        double slowingDistance = distanceToGoalSpeed(vMax);
        double distToGo = distanceRemaining();
        
        // Ramp velocity
        if (clearPath()) {
            if (slowingDistance < distToGo) {   // go to maximum velocity
                speedNominal = min(speedNominal + aMax * REFRESH_RATE, vGoal);
            } else {    // ramp speed down to goal speed
                speedNominal = max(speedNominal - aMax * REFRESH_RATE, 0.0);
            }
        } else {
            // go to zero
            speedNominal = max(speedNominal - aMax * REFRESH_RATE, 0.0);
        }
                
        // set only the speedNominal of the crawlerDesState
        curState.speedNominal = speedNominal;
        pub.publish(curState); // publish the crawlerDesState
	    
	    naptime.sleep(); // enforce desired update rate
    }
    return 0;   // this code will only get here if this node was told to shut down, which is
                // reflected in ros::ok() is false
}
