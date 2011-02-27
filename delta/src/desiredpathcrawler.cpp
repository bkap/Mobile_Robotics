#include<ros/ros.h>
#include<math.h>
#include<iostream>
#include<geometry_msgs/Pose.h> //data type for Pose combined with frame and timestamp
#include<tf/transform_datatypes.h> // for tf::getYaw
#include<tf/transform_listener.h> // for the TransformListener class that abstracts away a lot of tf
#include <eecs376_msgs/PathSegment.h>
#include <eecs376_msgs/PathList.h>
#include <eecs376_msgs/CrawlerDesiredState.h>
#include "command_publisher.h"

using namespace eecs376_msgs;
using namespace std;
const double REFRESH_RATE = 0.1;
const double pi = 3.141592;

PathList pathlist;
CrawlerDesiredState desState;

bool pathListInit = false;
bool speedProfilerInit = false;

void pathListCallback(const PathList::ConstPtr& newPathList)
{
    pathlist = *newPathList;
    pathListInit = true;
}

void speedProfilerCallback(const CrawlerDesiredState::ConstPtr& oldDesState)
{
    desState = *oldDesState;
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
    ros::Publisher pub = n.advertise<CrawlerDesiredState>("crawlerDesState",1);
    
    // list of subscribers
    ros::Subscriber subPathList = n.subscribe<PathList>("pathList", 1, pathListCallback);
	ros::Subscriber subSpeedProfiler = n.subscribe<CrawlerDesiredState>("NominalSpeed", 1, speedProfilerCallback);
	
	ros::Rate naptime(1/REFRESH_RATE); //will perform sleeps to enforce loop rate of "10" Hz
    while (!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly.
    //Wait until ros::Time::now() will be valid, but let any callbacks happen
    
    while(!pathListInit){ros::spinOnce();naptime.sleep();}

    // initialize desired state
    desState.seg_type = pathlist.path_list[0].seg_type;
    desState.seg_number = 0;
    desState.des_speed = 0.0;
    desState.des_rho = pathlist.path_list[0].curvature;
    desState.des_lseg = 0.0;
    geometry_msgs::Pose des_pose;
    des_pose.position = pathlist.path_list[0].ref_point;
    des_pose.orientation = pathlist.path_list[0].init_tan_angle;
    desState.des_pose = des_pose;
    
    // ref
    bool finishedPath = false;

    //ros::Time refTime = ros::Time::now();

    while (ros::ok()) // do work here
    {
        ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely
        
        // only update if it still has path segments left...
        if (!finishedPath)
        {
            // update total distance traveled
            double olddist = desState.des_lseg;
            //ros::Duration elapsed_time = ros::Time::now() - refTime;
            desState.des_lseg = desState.des_lseg + desState.des_speed * REFRESH_RATE;
            //desState.des_lseg = desState.des_lseg + desState.des_speed * elapsed_time.toSec();
            //cout << elapsed_time.toSec();
            //refTime = ros::Time::now();
            cout << "\ndpc: now traveled " << desState.des_lseg << " = " << olddist << " + " << desState.des_speed << "*" << REFRESH_RATE;
            
            // compute the x, y, psi
            double psiDes = tf::getYaw(desState.des_pose.orientation);
            
            switch (desState.seg_type) {
                case 1: // line
                    // straightforward
                    cout << "\nx = " << desState.des_pose.position.x << " + " << desState.des_lseg << " * " << cos(psiDes);
                    desState.des_pose.position.x += desState.des_lseg * cos(psiDes);
                    desState.des_pose.position.y += desState.des_lseg * sin(psiDes);
                    break;
                case 2: // arc: speedNominal is the tangential velocity (v = w R)
                {   // lsegdes is distance s traveled along the arc: s = R * theta
                    double theta = 0.0; // angle from center of curvature
                    if (desState.des_rho >= 0)
                        theta = psiDes - pi/2;
                    else
                        theta = psiDes + pi/2;
                    theta = theta + desState.des_lseg * fabs(desState.des_rho);
                    psiDes = psiDes + desState.des_lseg * fabs(desState.des_rho);
                    
                    desState.des_pose.position.x += cos(theta) / desState.des_rho;
                    desState.des_pose.position.y += sin(theta) / desState.des_rho;
                    break;
                }
                case 3: // rotate about point
                    // "distance" is actually the angle rotated, x and y do not change
                    psiDes += desState.des_lseg;
                    break;
            }
            
            // fix heading
            if (psiDes > pi)
                psiDes -= 2*pi;
            else if (psiDes < -pi)
                psiDes += 2*pi;
            
            desState.des_pose.orientation = tf::createQuaternionMsgFromYaw(psiDes);
            
            // figure out if we are at a new segment
            if (desState.des_lseg >= pathlist.path_list[desState.seg_number].seg_length) {
                // if this is not the last segment, increment
                if (desState.seg_number < pathlist.path_list.size()-1) {
                    // reset the desState
                    desState.seg_number++;
                    desState.seg_type = pathlist.path_list[desState.seg_number].seg_type;
                    // desState.des_speed = 0.0; // this will be set in profiler, don't change
                    desState.des_rho = pathlist.path_list[desState.seg_number].curvature;
                    desState.des_lseg = 0.0;
                    geometry_msgs::Pose des_pose;
                    des_pose.position = pathlist.path_list[desState.seg_number].ref_point;
                    des_pose.orientation = pathlist.path_list[desState.seg_number].init_tan_angle;
                    desState.des_pose = des_pose;
                } else {
                    cout << "\ndpc: LAST segment";
                    // desState.seg_number == pathlist.size() - 1 (last element of array)
                    finishedPath = true;
                }
                desState.seg_type = pathlist.path_list[desState.seg_number].seg_type;
                cout << "\n\ndpc: NEW SEGMENT type " << (int)desState.seg_type;
            }
        }
        cout << "\ndpc: x = " << desState.des_pose.position.x << ", y=" << desState.des_pose.position.y;
        cout << ", psi=" << tf::getYaw(desState.des_pose.orientation) << ", des_speed=" << desState.des_speed;
        cout << " (traveled " << desState.des_lseg << " which is " << pathlist.path_list[desState.seg_number].seg_length;
        cout << ") of type " << (int)desState.seg_type << "\n";
        
        pub.publish(desState); // publish the CrawlerDesiredState (if the path is over, doesn't change)
	    naptime.sleep(); // enforce desired update rate
    }
    return 0;   // this code will only get here if this node was told to shut down, which is
                // reflected in ros::ok() is false
}
