#include<ros/ros.h>
#include<geometry_msgs/Twist.h> //data type for velocities
#include<math.h>
#include <iostream>
#include <algorithm>
using namespace std;
const double MAX_SPEED = 2.0;
const double MAX_ROTATE = 2.0;
const double MAX_ACCEL = 2.0;
const double REFRESH_RATE = 0.1;
const double MAX_ANGLE_ACCEL = 1.0;
double getRobotVelocity(double cur_vel, double distance_to_dest) {
  double distance_if_decel = 0.5 * cur_vel * (cur_vel / (MAX_ACCEL * REFRESH_RATE));
  if(distance_to_dest - distance_if_decel < 0.2) {
    return max(cur_vel - MAC_ACCEL * REFRESH_RATE, min(0.2,distance_to_dest));
  } else if(cur_vel < MAX_SPEED) {
    return cur_vel + MAX_ACCEL * REFRESH_RATE;
  } else {
    return cur_vel;
  }
}

double goDistance(double *velocity, double distance, double time_period) {
  *velocity = getRobotVelocity(*velocity, distance);
  double distance_returned = distance - *velocity * time_period;
  return distance_returned;
}

double getRobotRotation(double cur_rotate, double remaining_rotate) {
  double rotate_if_decel = -0.5 * cur_rotate * (cur_rotate / (REFRESH_RATE * MAX_ANGLE_ACCEL));
  if(rotate_if_decel - remaining_rotate < -0.1) {
    return min(cur_rotate + (REFRESH_RATE * MAX_ANGLE_ACCEL), max(remaining_rotate, -0.1));
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
	ros::Duration run_duration(22.0); // specify desired duration of this command segment to be 3 seconds
	ros::Duration elapsed_time; // define a variable to hold elapsed time
	ros::Rate naptime(10); //will perform sleeps to enforce loop rate of "10" Hz
	while (!ros::Time::isValid()) {} // simulation time sometimes initializes slowly. Wait until ros::Time::now() will be valid
	ros::Time birthday= ros::Time::now(); // get the current time, which defines our start time, called "birthday"
	ROS_INFO("birthday started as %f", birthday.toSec());
  int stage = 0;
  double amounts_to_change[] = {3.0,asin(-1),12.2,asin(-1),4,-1};

  double amount_to_change = 0.0;
	while (ros::ok()) // do work here
	{
		elapsed_time= ros::Time::now()-birthday;
		ROS_INFO("birthday is %f", birthday.toSec());
		ROS_INFO("elapsed time is %f", elapsed_time.toSec());
    cout << amount_to_change <<endl;
    if(fabs(amount_to_change) <=  0.02) {
      cout << "Increasing Stage";
      amount_to_change = amounts_to_change[stage];
      stage++;
      vel_object.linear.x = 0.0;
      vel_object.angular.z = 0.0;
    }
    if(stage == 1 || stage == 3  || stage == 5) {
      amount_to_change = goDistance(&(vel_object.linear.x),amount_to_change,0.1);
       
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
