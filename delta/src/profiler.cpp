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

// TODO: these are not actually constant they are part of the path lolz
const double MAX_LINEAR_VEL = 1.0;
const double MAX_LINEAR_ACCEL = 2.0;
const double MAX_ANG_VEL = 1.0;
const double MAX_ANG_ACCEL = 1.0;

// TODO: get these from subscribed topics somehow
double cur_x, cur_y, cur_psi, xDes, yDes, psiDes, rhoDes, lsegDes, goal_lin_vel, goal_ang_vel;
int segtype;

// keep track of last ordered velocities
double cur_lin_vel = 0.0;
double cur_ang_vel = 0.0;

// TODO: all of these signatures are wrong but I don't know what to put here
void crawlerDesStateCallback(const CrawlerDesiredState::ConstPtr& desState)
{
    /*// get cur_x, cur_y, cur_psi    
    cur_x = 0.0;
    cur_y = 0.0;
    cur_psi = 0.0;
    
    lsegDes = 0.0; // how far traveled along current path segment
    xDes = 0.0;
    yDes = 0.0;
    psiDes = 0.0;
    rhoDes = 0.0;
    goal_lin_vel = 0.0;
    goal_ang_vel = 0.0;
    segtype = 0;*/

}

void pathListCallback(const PathList::ConstPtr& path)
{
    // get the path segment type
    // get xDes, yDes, psiDes, rhoDes, goal_lin_vel, goal_ang_vel, max velocities/accelerations
}

//http://www.ros.org/doc/api/nav_msgs/html/msg/OccupancyGrid.html
void cspaceMapCallback(const nav_msgs::OccupancyMap::ConstPtr& CSpaceMap)
{
    // get the cspacemap
}

// Calculate the remaining distance to the end of the path segment
double distanceRemaining()
{
    switch (segtype) {
        case 1: // line
            return pow(pow(xDes-cur_x, 2) + pow(yDes-cur_y, 2), 0.5);
        case 2: // arc
            return rhoDes * (psiDes - cur_psi);
        case 3: // rotate in place - no translation
            return 0.0;
    }
}

// Calculate the remaining angle to the end of the path segment
double angleRemaining()
{
    switch (segtype) {
        case 1: // line - no rotation
            return 0.0;
        case 2: // arc
            return psiDes - cur_psi;
        case 3: // rotate in place - distance is the angle left to turn
            return psiDes - cur_psi;
    }
}

// TODO: Check if path is clear within the braking distances
bool clearPath(double dist_lin, double dist_ang)
{
    // check along the path within braking distances
    // and test cspace map pixels for occupancy
    return true;
}

/*
Subscribes: CrawlerDesState, pathList, CspaceMap (s)

Behavior: start at current point on pathList
if path is clear:
    if room, speed up
    if within stopping distance, slow down
if obstruction:
    brake

Publishes: speednominal?
a eecs376_msgs/CrawlerDesiredState, but with the des_speed field filled in with a correct value
*/
int main(int argc,char **argv)
{   
    ros::init(argc,argv,"profiler");//name of this node
    CrawlerDesState desState; // intialize the class

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<CrawlerDesState>("speedprofiler",1);
    
    // list of subscribers
    // TODO: not sure if these are the right strings, also these are the wrong classes for pathlist and cspacemap
	ros::Subscriber subCrawlerDesState = n.subscribe<geometry_msgs::Pose>("crawlerDesState", 1, crawlerDesStateCallback);
	ros::Subscriber subPathList = n.subscribe<PathList>("pathList", 1, pathListCallback);
	ros::Subscriber subCspaceMap = n.subscribe<CrawlerDesState>("cspaceMap", 1, cspaceMapCallback);
	
	ros::Rate naptime(1/REFRESH_RATE); //will perform sleeps to enforce loop rate of "10" Hz
    while (!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly.
    //Wait until ros::Time::now() will be valid, but let any callbacks happen
    
    // Calculate braking distances
    double brakingDistanceLin = cur_lin_vel * (cur_lin_vel - goal_lin_vel) / MAX_LINEAR_ACCEL - pow(cur_lin_vel - goal_lin_vel, 2) / (2 * MAX_LINEAR_ACCEL);
    // this is an angle
    double brakingDistanceAng = cur_ang_vel * (cur_ang_vel - goal_ang_vel) / MAX_ANG_ACCEL - pow(cur_ang_vel - goal_ang_vel, 2) / (2 * MAX_ANG_ACCEL);
    
    while (ros::ok()) // do work here
    {
        ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely
        
        // Ramp linear velocity
        if (distanceRemaining() < brakingDistanceLin && clearPath(brakingDistanceLin, brakingDistanceAng)) {
            // go to maximum velocity
            if (cur_lin_vel < MAX_LINEAR_VEL)
                cur_lin_vel = min(cur_lin_vel + MAX_LINEAR_ACCEL * REFRESH_RATE, MAX_LINEAR_VEL);
        } else {
            // go to zero
            cur_lin_vel = max(cur_lin_vel - MAX_LINEAR_ACCEL * REFRESH_RATE, 0.0);
        }
        
        // TODO: Ramp angular velocity... what is this I don't even (aka this probably isn't right and is going to spiral weirdly?)
        if (angleRemaining() < brakingDistanceAng && clearPath(brakingDistanceLin, brakingDistanceAng)) {
            // go to maximum velocity
            if (cur_ang_vel < MAX_ANG_VEL)
                cur_ang_vel = min(cur_ang_vel + MAX_ANG_ACCEL * REFRESH_RATE, MAX_ANG_VEL);
        } else {
            // go to zero
            cur_ang_vel = max(cur_ang_vel - MAX_ANG_ACCEL * REFRESH_RATE, 0.0);
        }
        
        // set only the speedNominal of the crawlerDesState
        // speedNominal is the only thing to update
        // um.  v = w/R...?
        //pub.publish(goal_pose); // publish the crawlerDesState
	    
	    naptime.sleep(); // enforce desired update rate
    }
    return 0;   // this code will only get here if this node was told to shut down, which is
                // reflected in ros::ok() is false
}
