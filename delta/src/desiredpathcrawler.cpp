#include<ros/ros.h>
#include<math.h>
#include<iostream>
#include<geometry_msgs/Pose.h> //data type for Pose combined with frame and timestamp
#include<tf/transform_datatypes.h> // for tf::getYaw
#include<tf/transform_listener.h> // for the TransformListener class that abstracts away a lot of tf
#include <eecs376_msgs/PathSegment.h>
#include <eecs376_msgs/PathList.h>
#include "command_publisher.h"

using namespace eecs376_msgs
using namespace std;
const double REFRESH_RATE = 0.1;

double speedNominal, segLen, referencePtX, referencePtY, initTanAng, rho, maxVelLin, maxAccelLin, maxVelAng, maxAccelAng;
int segNum, segType;

// TODO: all of these signatures are wrong but I don't know what to put here
void pathListCallback(const PathList::ConstPtr& pathlist)
{
    // get the path segment type
    // get xDes, yDes, psiDes, rhoDes, goal_lin_vel, goal_ang_vel, max velocities/accelerations
    segNum = 0;         // index of path segment in path list
    segType = 0;        // 1=path, 2=arc, 3=rotate in place
    segLen = 0.0;       // length of path, arc, or angle through which to rotate
    referencePtX = 0.0; // if line, start of path, else, center of rotation
    referencePtY = 0.0;
    initTanAng = 0.0;   // initial heading
    rhoDes = 0.0;          // curvature (0 for lines)
    maxVelLin = 0.0;
    maxAccelLin = 0.0;
    maxVelAng = 0.0;
    maxAccelAng = 0.0;
}

void speedNominalCallback(const CrawlerDesState::ConstPtr& desState)
{
    // get speedNominal
    speedNominal = 0.0;
}

/*
Subscribes: speedNominal (speedProfiler), pathList

uses v*dt to update lsegdes; compute x, y, psi, rho

Publishes: CrawlerDesiredState
lsegDes (how far along current segment you've gone), xDes, yDes, psiDes, rhoDes, segType, segNumber
*/
int main(int argc,char **argv)
{   
    ros::init(argc,argv,"desiredpathcrawler");//name of this node
    
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<CrawlerDesState>("desiredpathcrawler",1);
    
    // list of subscribers
    // TODO: probably wrong types
	ros::Subscriber subPathList = n.subscribe<PathList>("pathList", 1, pathListCallback);
	ros::Subscriber subSpeedProfiler = n.subscribe<CrawlerDesState>("speedprofiler", 1, speedProfilerCallback);
	
	ros::Rate naptime(1/REFRESH_RATE); //will perform sleeps to enforce loop rate of "10" Hz
    while (!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly.
    //Wait until ros::Time::now() will be valid, but let any callbacks happen
       
    while (ros::ok()) // do work here
    {
        ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely
        
        // update total distance traveled
        lsegDes = lsegDes + speedNominal * REFRESH_RATE;
        
        // compute the x, y, psi
        switch (segType) {
            case 1: // line
                // straightforward
                xDes = xDes + lsegDes * cos(psiDes);
                yDes = yDes + lsegDes * sin(psiDes);
                break;
            case 2: // arc: speedNominal is the tangential velocity (v = w R)
                // lsegdes is distance s traveled along the arc: s = R * theta
                double theta = 0.0; // angle from center of curvature
                if (rho >= 0)
                    theta = psiDes - pi/2;
                else
                    theta = psiDes + pi/2;
                theta = theta + lsegDes * fabs(rhoDes)
                psiDes = psiDes + lsegDes * fabs(rhoDes);
                
                xDes = xDes + cos(theta)/rhoDes;
                yDes = yDes + sin(theta)/rhoDes;
                break;
            case 3: // rotate about point
                // "distance" is actually the angle rotated, x and y do not change
                psiDes = psiDes + lsegDes;
                break;
        }
        
        // figure out if we are at a new segment
        if (lsegDes >= segLen) {  // TODO: add tolerance?
            segNum++;
            segType = pathList[segNum].segType //TODO: haha this is wrong
        }
        
        pub.publish(CrawlerDesState); // publish the crawlerDesState
	    naptime.sleep(); // enforce desired update rate
    }
    return 0;   // this code will only get here if this node was told to shut down, which is
                // reflected in ros::ok() is false
}
