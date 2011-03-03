#include<ros/ros.h>
#include<math.h>
#include<iostream>
#include<geometry_msgs/Pose.h> //data type for Pose combined with frame and timestamp
#include<tf/transform_datatypes.h> // for tf::getYaw
#include<tf/transform_listener.h> // for the TransformListener class that abstracts away a lot of tf

using namespace std;
const double REFRESH_RATE = 0.1;

/*
set and update goal pose(s)
goal pose can be hard-coded and set once or updated frequently
goal pose can also be a vector of equivalent, alternative goals

This returns a single goal pose that is hard-coded
*/
int main(int argc,char **argv)
{
    ros::init(argc,argv,"goalpublisher");//name of this node
  	geometry_msgs::Pose goal_pose;
	
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Pose>("goalPose",1); //topic name "goal_pub", buffer size 1
    
    ros::Duration elapsed_time; // define a variable to hold elapsed time
    ros::Rate naptime(1/REFRESH_RATE); //will perform sleeps to enforce loop rate of "10" Hz
    while (!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly.
    //Wait until ros::Time::now() will be valid, but let any callbacks happen
    
    while (ros::ok()) // do work here
    {
        ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely
        // set x, y, and heading
     	goal_pose.position.x = 0.0;
      	goal_pose.position.y = 24.0;
      	goal_pose.orientation = tf::createQuaternionMsgFromYaw(47.83);
        //cout<<"\nhey hey listen " << goal_pose.position.x << ", " << goal_pose.position.y;
	    pub.publish(goal_pose);
	    naptime.sleep(); // enforce desired update rate
    }
    return 0;   // this code will only get here if this node was told to shut down, which is
                // reflected in ros::ok() is false
}
