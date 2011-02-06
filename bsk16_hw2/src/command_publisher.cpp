#include<ros/ros.h>
#include<geometry_msgs/Twist.h> //data type for velocities
#include<math.h>
#include <iostream>
#include <algorithm>
#include<geometry_msgs/PoseStamped.h> //data type for Pose combined with frame and timestamp
#include<nav_msgs/Odometry.h> //data type for odometry information (see available fields with 'rosmsg show nav_msgs/Odometry')
#include<tf/transform_datatypes.h> // for tf::getYaw
#include<tf/transform_listener.h> // for the TransformListener class that abstracts away a lot of tf

using namespace std;
const double MAX_SPEED = 2.0;
const double MAX_ROTATE = 2.0;
const double MAX_ACCEL = 2.0;
const double REFRESH_RATE = 0.05;
const double MAX_ANGLE_ACCEL = 1.0;


nav_msgs::Odometry last_odom;
geometry_msgs::PoseStamped last_map_pose;
tf::TransformListener *tfl;

geometry_msgs::PoseStamped temp;
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) 
{
        last_odom = *odom;
        temp.pose = last_odom.pose.pose;
        temp.header = last_odom.header;
        try 
	{
          tfl->transformPose("map", temp, last_map_pose);
        } 
	catch (tf::TransformException ex) 
	{
          ROS_ERROR("%s", ex.what());
        }
}

double getRobotVelocity(double cur_vel, double distance_to_dest) {
  double distance_if_decel = 0.5 * cur_vel * (cur_vel / (MAX_ACCEL * REFRESH_RATE));
  if(distance_to_dest - distance_if_decel < 0.2) {
    return max(cur_vel - MAX_ACCEL * REFRESH_RATE, min(0.2,distance_to_dest));
  } else if(cur_vel < MAX_SPEED) {
    return cur_vel + MAX_ACCEL * REFRESH_RATE;
  } else {
    return cur_vel;
  }
}

// this method needs to take the desired_pose and 
void calculateSteeringRotation(double *rotation, geometry::PoseStamped* desired_pose, double distance) {
	desired_pose.pose.position.x += distance * cos(tf::getYaw(desired_pose.pose.orientation));
	desired_pose.pose.position.y += distance * sin(tf::getYaw(desired_pose.pose.orientation));
	//TODO: actually have this method do stuff
	// take the last_map_pose (current pose) vs desired_pose (what we want), in same frame
	
	// constants
	double kd = 1.0;
	double kw = 1.0;
	double x_err = desired_pose.pose.position.x - last_map_pose.pose.pose.x;
	double y_err = desired_pose.pose.position.y - last_map_pose.pose.pose.y;
	double w_err = getYaw(desired_pose.pose.orientation) - getYaw(last_map_pose.pose.orientation);
	
	// omega = rotation = - err_d kd - err_w kw
	(*rotation) = - kd*x_err - kd*y_err - kw*w_err; // how far we are off in radians
}
double goDistance(double *velocity, double *rotation, geometry::PoseStamped* desired_pose, double distance, double time_period) {
  *velocity = getRobotVelocity(*velocity, distance);
  double distance_returned = distance - *velocity * time_period;
	calculateSteeringRotation(rotation, desired_pose, distance);
  return distance_returned;
}

double getRobotRotation(double cur_rotate, double remaining_rotate) {
  double rotate_if_decel = -0.5 * cur_rotate * (cur_rotate / (REFRESH_RATE * MAX_ANGLE_ACCEL));
  if(rotate_if_decel - remaining_rotate < -0.1) {
    return min(cur_rotate + (REFRESH_RATE * MAX_ANGLE_ACCEL), max(remaining_rotate * 2, -0.1));
  } else if(fabs(cur_rotate) < MAX_ROTATE) {
    return cur_rotate - MAX_ANGLE_ACCEL * REFRESH_RATE;
  } else {
    return cur_rotate;
  }
}

double goRotate(double *rotation, double rotate, double time_period) {
  *rotation = getRobotRotation(*rotation, rotate);
   double rotate_returned = rotate  - *rotation * time_period;
  return rotate_returned;

}
int main(int argc,char **argv)
{
	ros::init(argc,argv,"command_publisher");//name of this node
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	//"cmd_vel" is the topic name to publish velocity commands
	//"1" is the buffer size (could use buffer>1 in case network bogs down)

	//send robot forward for 3 seconds, reiterated at 10Hz.  Need a ROS "rate" object to enforce a rate
	geometry_msgs::Twist vel_object;
	ros::Duration elapsed_time; // define a variable to hold elapsed time
	ros::Rate naptime(REFRESH_RATE * 1000); //will perform sleeps to enforce loop rate of "10" Hz
        geometry_msgs::PoseStamped desired_pose;
	
	while (!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly. Wait until ros::Time::now() will be valid, but let any callbacks happen
        
	while (!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce(); // wait until there is transform data available before starting our controller loopros::Time birthday= ros::Time::now(); // get the current time, which defines our start time, called "birthday"
	ROS_INFO("birthday started as %f", birthday.toSec());
  int stage = 0;
  double amounts_to_change[] = {3.0,asin(-1),12.2,asin(-1),4,-1};

      double amount_to_change = 0.0;    
      desired_pose.header.stamp = current_time;
      desired_pose.header.frame_id = "map";
      desired_pose.pose.position.x = -13.6;
      desired_pose.pose.position.y = -24.89;
      desired_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-2.38342);
	while (ros::ok()) // do work here
	{
		elapsed_time= ros::Time::now()-birthday;
		ROS_INFO("birthday is %f", birthday.toSec());
		ROS_INFO("elapsed time is %f", elapsed_time.toSec());
    cout << amount_to_change <<endl;
    if(fabs(amount_to_change) <=  0.01) {
      cout << "Increasing Stage";
      amount_to_change = amounts_to_change[stage];
      stage++;
      vel_object.linear.x = 0.0;
      vel_object.angular.z = 0.0;

      ROS_INFO("odom = x: %f, y: %f, heading: %f", last_odom.pose.pose.position.x, last_odom.pose.pose.position.y, tf::getYaw(last_odom.pose.pose.orientation));
      ROS_INFO("map pose = x: %f, y: %f, heading: %f", last_map_pose.pose.position.x, last_map_pose.pose.position.y, tf::getYaw(last_map_pose.pose.orientation));

      desired_pose.header.stamp = current_time;
      des_pose_pub.publish(desired_pose);

    }
    if(stage == 1 || stage == 3  || stage == 5) {
      amount_to_change = goDistance(&(vel_object.linear.x), &(vel_object.angular.z), &desired_pose, amount_to_change,0.1);
       
    } else if(stage == 2 || stage == 4) {
      amount_to_change = goRotate(&(vel_object.angular.z),amount_to_change, 0.1);
		}

    if(stage < 6) {
			// send out new command appropriate for this instant;
			// boring--always send out the same speed/spin values in this example
			pub.publish(vel_object);  // this action causes the commands in vel_object to be published 
		}
		else
		{
			// done with 3 seconds of commands; now send out zeros indefinitely to halt the robot
			vel_object.linear.x = 0.0;
			vel_object.angular.z = 0.0;
			pub.publish(vel_object);
		}
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (100ms) period
		//thus enforcing that we achieve the desired update rate (10Hz)
	}
	return 0; // this code will only get here if this node was told to shut down, which is
	// reflected in ros::ok() is false 
}
