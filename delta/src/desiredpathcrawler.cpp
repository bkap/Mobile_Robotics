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
const double REFRESH_RATE = 0.05;
const double pi = 3.141592;

PathList pathlist;
CrawlerDesiredState desState;

bool pathListInit = false;
bool speedProfilerInit = false;

void pathListCallback(const PathList::ConstPtr& newPathList)
{
    pathlist = *newPathList;
    // hax to make it turn 1/2 the desired angle on arcs because this is probably wrong elsewhere
    /*for (int i=0; i<pathlist.path_list.size(); i++)
        if (pathlist.path_list[i].seg_type == 2)
            pathlist.path_list[i].seg_length = pathlist.path_list[i].seg_length / 2;
    */pathListInit = true;
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
            // update total distance traveled and compute the x, y, psi

            double psiDes = tf::getYaw(desState.des_pose.orientation);            
            switch (desState.seg_type) {
                case 1: // line
                    // straightforward
                    desState.des_lseg += desState.des_speed * REFRESH_RATE; // v_linear
                    desState.des_pose.position.x += desState.des_speed * REFRESH_RATE * cos(psiDes);
                    desState.des_pose.position.y += desState.des_speed * REFRESH_RATE * sin(psiDes);
                    break;
                case 2: // arc: speedNominal is the tangential velocity (v = w R), des_lseg is angle
                {
                    //distance the robot traveled in the past dt along the arc.  s = R*theta
                    desState.des_lseg += desState.des_speed * fabs(desState.des_rho) * REFRESH_RATE;
                    
                    cout << "\nARCING around circle centered at " << pathlist.path_list[desState.seg_number].ref_point << " with dpsi " << pathlist.path_list[desState.seg_number].seg_length;
                    
                    double theta = 0.0; // angle from center of curvature
                    ///*
                    if (desState.des_rho >= 0) {
                        cout << "\nLEFT TURN";
                        theta = psiDes - pi/2;
                    } else { // right hand
                        cout << "\nRIGHT TURN";
                        theta = psiDes + pi/2;
                    }
                    //*/
                    //theta = psiDes + pi/2;
                    if (desState.des_rho < 0) {  // right-hand turn
                        //cout << "\nRHT theta = "<<theta;
                        theta = theta - desState.des_speed * REFRESH_RATE * fabs(desState.des_rho);
                        psiDes = psiDes - desState.des_speed * REFRESH_RATE * fabs(desState.des_rho);
                    } else {    // left-hand turn
                        //cout << "\nLHT theta = "<<theta;
                        theta = theta + desState.des_speed * REFRESH_RATE * fabs(desState.des_rho);
                        psiDes = psiDes + desState.des_speed * REFRESH_RATE * fabs(desState.des_rho);
                    }
                    
                    // merely for reference
                    double xCenter = pathlist.path_list[desState.seg_number].ref_point.x;
                    double yCenter = pathlist.path_list[desState.seg_number].ref_point.y;
                    
                    // this was the old stuff that didn't work
                    //desState.des_pose.position.x = xCenter + cos(theta) / desState.des_rho;
                    //desState.des_pose.position.y = yCenter + sin(theta) / desState.des_rho;
                    
                    desState.des_pose.position.x = xCenter + cos(theta - pi) / desState.des_rho;
                    desState.des_pose.position.y = yCenter + sin(theta - pi) / desState.des_rho;
                    
                    break;
                }
                case 3: // rotate about point
                    desState.des_lseg += desState.des_speed * REFRESH_RATE; // v_angular
                    // "distance" is actually the angle rotated, x and y do not change
                    if (desState.des_rho < 0) // right-hand
                        psiDes -= desState.des_speed * REFRESH_RATE;
                    else // left-hand
                        psiDes += desState.des_speed * REFRESH_RATE;
                    break;
            }
            
            // fix heading
            if (psiDes > pi)
                psiDes -= 2*pi;
            else if (psiDes < -pi)
                psiDes += 2*pi;
            
            desState.des_pose.orientation = tf::createQuaternionMsgFromYaw(psiDes);
            
            // figure out if we are at a new segment
            if (desState.des_lseg >= fabs(pathlist.path_list[desState.seg_number].seg_length)) {
                // if this is not the last segment, increment
                if (desState.seg_number < pathlist.path_list.size()-1) {
                    // reset the desState
                    desState.seg_number++;
                    desState.seg_type = pathlist.path_list[desState.seg_number].seg_type;
                    // desState.des_speed = 0.0; // this will be set in profiler, don't change
                    desState.des_rho = pathlist.path_list[desState.seg_number].curvature;
                    desState.des_lseg = 0.0;
                    geometry_msgs::Pose des_pose;
                    des_pose.orientation = pathlist.path_list[desState.seg_number].init_tan_angle;
                    
                    if (desState.seg_type == 2) { // arc
                        double theta = 0.0; // angle from center of curvature
                        
                        if (desState.des_rho >= 0)
                            theta = psiDes - pi/2;
                        else
                            theta = psiDes + pi/2;
                        
                        /*if (desState.des_rho < 0) {  // right-hand turn
                            cout << "\nRHT theta = "<<theta;
                            theta = theta - desState.des_speed * REFRESH_RATE * fabs(desState.des_rho);
                            psiDes = psiDes - desState.des_speed * REFRESH_RATE * fabs(desState.des_rho);
                        } else {    // left-hand turn
                            cout << "\nLHT theta = "<<theta;
                            theta = theta + desState.des_speed * REFRESH_RATE * fabs(desState.des_rho);
                            psiDes = psiDes + desState.des_speed * REFRESH_RATE * fabs(desState.des_rho);
                        }*/
                        
                        double xCenter = pathlist.path_list[desState.seg_number].ref_point.x;
                        double yCenter = pathlist.path_list[desState.seg_number].ref_point.y;
                    
                        // this isn't really necessary
                        geometry_msgs::Point arcStartPt;
                        arcStartPt.x = xCenter + cos(theta - pi) / desState.des_rho;    // need to correct by pi
                        arcStartPt.y = yCenter + sin(theta - pi) / desState.des_rho;

                        /*cout << "\nARCING around " << xCenter <<","<<yCenter << " with rho=" << desState.des_rho << " ie radius=" << (1/desState.des_rho) << " psi="<<psiDes<<" theta="<<theta;
                        cout << "\npsiDes-pi: " << x2 << ", " << y2;
                        cout << "\ntheta+pi: " << x3<<", "<<y3;
                        cout << "\ntheta+3pi/4: " << x4<<", "<<y4;
                        cout << "\npsiDes: " << x5<<", "<<y5;
                        */
                        des_pose.position = arcStartPt;
                    } else { // line or point
                        des_pose.position = pathlist.path_list[desState.seg_number].ref_point;                    
                    }
                    
                    desState.des_pose = des_pose;
                } else {
                    cout << "\n\ndpc: LAST segment\n";
                    // desState.seg_number == pathlist.size() - 1 (last element of array)
                    finishedPath = true;
                }
                desState.seg_type = pathlist.path_list[desState.seg_number].seg_type;
                cout << "\n\ndpc: NEW SEGMENT type " << (int)desState.seg_type;
            }
        }
        /*cout << "\ndpc: s"<< desState.seg_number << " x = " << desState.des_pose.position.x << ", y=" << desState.des_pose.position.y;
        cout << ", psi=" << tf::getYaw(desState.des_pose.orientation) << ", des_speed=" << desState.des_speed;
        cout << " (traveled " << desState.des_lseg << " out of " << pathlist.path_list[desState.seg_number].seg_length;
        cout << ") of type " << (int)desState.seg_type << "\n";
        */
        pub.publish(desState); // publish the CrawlerDesiredState (if the path is over, doesn't change)
	    naptime.sleep(); // enforce desired update rate
    }
    return 0;   // this code will only get here if this node was told to shut down, which is
                // reflected in ros::ok() is false
}
