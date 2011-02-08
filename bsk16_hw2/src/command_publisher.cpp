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
const double REFRESH_RATE = 0.1;
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
	  cout << "We caught an error!" << endl;
          ROS_ERROR("%s", ex.what());
        }
}

void updateBreadCrumb(geometry_msgs::PoseStamped* last_desired_pose, geometry_msgs::PoseStamped* desired_pose, geometry_msgs::PoseStamped* bread_crumb, double vx)
{
	double dx = (*desired_pose).pose.position.x -(*last_desired_pose).pose.position.x;
	double dy = (*desired_pose).pose.position.y -(*last_desired_pose).pose.position.y;
	(*bread_crumb).pose.position.x -= vx*dx/sqrt(dx*dx+dy*dy)*REFRESH_RATE; //advance breadcrumb v*dt in the direction of desired pose.
        (*bread_crumb).pose.position.y -= vx*dy/sqrt(dx*dx+dy*dy)*REFRESH_RATE;
}

// update desired_pose based on radius of curvature and distance of next path segment
void updateDesiredPose(geometry_msgs::PoseStamped* last_desired_pose, geometry_msgs::PoseStamped* desired_pose, geometry_msgs::PoseStamped* bread_crumb, double curvature, double distance) 
{
	(*last_desired_pose) = (*desired_pose);
	if (curvature == 0) 
	{
		// go in a straight line for d m, keep current heading
    		cout << tf::getYaw((*desired_pose).pose.orientation)<< endl;
		 double dx = distance * cos(tf::getYaw((*desired_pose).pose.orientation));
		(*desired_pose).pose.position.x += dx;
     		double dy = distance * sin(tf::getYaw((*desired_pose).pose.orientation));
		(*desired_pose).pose.position.y += dy;
		double x = last_map_pose.pose.position.x - (*last_desired_pose).pose.position.x;
		double y = last_map_pose.pose.position.y - (*last_desired_pose).pose.position.y;

		double projx = (x*dx+y*dy)*dx/(sqrt(dx*dx+dy*dy)*sqrt(x*x+y*y));
		double projy = (x*dx+y*dy)*dy/(sqrt(dx*dx+dy*dy)*sqrt(x*x+y*y));	

    		cout << dx << "," << dy << endl;
		(*bread_crumb).pose.position.x = (*last_desired_pose).pose.position.x+projx+1*dx/sqrt(dx*dx+dy*dy); //start the bread crumb .5 meters in the direction of the goal.
		(*bread_crumb).pose.position.y = (*last_desired_pose).pose.position.y+projy+1*dy/sqrt(dx*dx+dy*dy);
	} 
	else 
	{
		// turn d radians in place, keep current position
		// TODO: actual arcs
		double newHeading = tf::getYaw((*desired_pose).pose.orientation) + distance;
		(*desired_pose).pose.orientation = tf::createQuaternionMsgFromYaw(newHeading);
	}
}

double getRobotVelocity(double cur_vel, double distance_to_dest) {
  double distance_if_decel = 0.5 * cur_vel * (cur_vel / (MAX_ACCEL));
  if(distance_to_dest - distance_if_decel < 0.2) {
    return max(cur_vel - MAX_ACCEL * REFRESH_RATE, min(0.2,distance_to_dest));
  } else if(cur_vel < MAX_SPEED) {
    return cur_vel + MAX_ACCEL * REFRESH_RATE;
  } else {
    return cur_vel;
  }
}

// this method needs to take the bread crumb and find the desired heading from that.
void calculateSteeringRotation(geometry_msgs::PoseStamped* bread_crumb, double *rotation, double distance) {
	
	//double kd = 0;
	double kw = 1;
	//double x_err = (*desired_pose).pose.position.x - last_map_pose.pose.position.x;
	//double y_err = (*desired_pose).pose.position.y - last_map_pose.pose.position.y;
	//double w_err = atan2(y_err, x_err) - tf::getYaw(last_map_pose.pose.orientation);
	//double w_error = tf::getYaw(last_map_pose.pose.orientation) - tf::getYaw((*desired_pose).pose.orientation);
	//double dx = (*desired_pose).pose.position.x;
	//double dy = (*desired_pose).pose.position.y;
	//double mx = last_map_pose.pose.position.x;
	//double my = last_map_pose.pose.position.y;
	//double PsiDP = atan((dx-mx)/(dy-my));
	//double Theta1 = w_error-PsiDP;
	//double PsiErr = -w_error;
	//double derp = sin(Theta1)*sqrt((dx-mx)*(dx-mx)+(dy-my)*(dy-my));
	
	// omega = rotation = - err_d kd - err_w kw
	//(*rotation) = kd*derp+kw*PsiErr;
	
	double dx = (*bread_crumb).pose.position.x-last_map_pose.pose.position.x;//I think that this is based off of the older steering algorithm, but with a bread crumb.
	double dy = (*bread_crumb).pose.position.y-last_map_pose.pose.position.y;
	double PsiErr = -1 *tf::getYaw(last_map_pose.pose.orientation) + atan2(dy,dx);
  cout << "Bread Crumb: " << (*bread_crumb).pose.position.x << "," << (*bread_crumb).pose.position.y << endl;
  cout << "Last Pose: " << last_map_pose.pose.position.x << "," << last_map_pose.pose.position.y << endl;
	(*rotation) = kw*PsiErr;
}

double goDistance(double *velocity, double *rotation, geometry_msgs::PoseStamped* bread_crumb, double distance, double time_period) {
  (*velocity) = getRobotVelocity(*velocity, distance);
  double distance_returned = distance - (*velocity) * time_period;
	calculateSteeringRotation(bread_crumb, rotation, distance);
  return distance_returned;
}

double getRobotRotation(double cur_rotate, double remaining_rotate) 
{
  double rotate_if_decel = -0.5 * cur_rotate * (cur_rotate / (REFRESH_RATE * MAX_ANGLE_ACCEL));
  if(rotate_if_decel - remaining_rotate < -0.1) {
    return min(cur_rotate + (REFRESH_RATE * MAX_ANGLE_ACCEL), max(remaining_rotate * 2, -0.1));
  } else if(fabs(cur_rotate) < MAX_ROTATE) {
    return cur_rotate - MAX_ANGLE_ACCEL * REFRESH_RATE;
  } else {
    return cur_rotate;
  }
}

double goRotate(double *rotation, double rotate, double time_period) 
{
  *rotation = getRobotRotation(*rotation, rotate);
   double rotate_returned = rotate  - (*rotation) * time_period;
  return rotate_returned;

}
int main(int argc,char **argv)
{
	
	ros::init(argc,argv,"command_publisher");//name of this node
	 
      	double amount_to_change = 0.0;    
      	geometry_msgs::PoseStamped desired_pose;
	geometry_msgs::PoseStamped bread_crumb;
	geometry_msgs::PoseStamped last_desired_pose; // for reference
      		
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Publisher des_pose_pub = n.advertise<geometry_msgs::PoseStamped>("desired_pose", 1);
	tfl = new tf::TransformListener();
	ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback); 
	//"cmd_vel" is the topic name to publish velocity commands
	//"1" is the buffer size (could use buffer>1 in case network bogs down)

	//send robot forward for 3 seconds, reiterated at 10Hz.  Need a ROS "rate" object to enforce a rate
	geometry_msgs::Twist vel_object;
	ros::Duration elapsed_time; // define a variable to hold elapsed time
	ros::Rate naptime(REFRESH_RATE * 1000); //will perform sleeps to enforce loop rate of "10" Hz
  cout << "sleeping : " << REFRESH_RATE * 1000;
	while (!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly. Wait until ros::Time::now() will be valid, but let any callbacks happen
      
 	desired_pose.header.frame_id = "map";
      	desired_pose.pose.position.x = 8.5;
      	desired_pose.pose.position.y = 15;
      	desired_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-2.38342);
	ros::Time birthday = ros::Time::now();
	desired_pose.header.stamp = birthday;
	while (!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce(); // wait until there is transform data available before starting our controller loopros::Time birthday= ros::Time::now(); // get the current time, which defines our start time, called "birthday"
	
	ROS_INFO("birthday started as %f", birthday.toSec());
	int stage = 0;
	double amounts_to_change[] = {3.3,asin(-1),12.4,asin(-1),4,-1};

	
	for(int i = 0; i < 10; i++)
	{
		cout<<"Waiting 10 cycles... "<<i<<"\n";
		ros::spinOnce();
	}

	while (ros::ok()) // do work here
	{
		ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely
		ros::Time current_time = ros::Time::now();
		desired_pose.header.stamp = current_time;
		elapsed_time= ros::Time::now()-birthday;
		ROS_INFO("birthday is %f", birthday.toSec());
		ROS_INFO("elapsed time is %f", elapsed_time.toSec());
		cout << amount_to_change <<endl;
		
		if(fabs(amount_to_change) <=  0.05) 
		{
			cout << "Increasing Stage";
			amount_to_change = amounts_to_change[stage];
			stage++;
			if(stage == 1 || stage == 3  || stage == 5)
			{
			      updateDesiredPose(&last_desired_pose, &desired_pose, &bread_crumb, 0, amount_to_change);
			}
			else if(stage == 2 || stage == 4) 
			{
			     updateDesiredPose(&last_desired_pose, &desired_pose, &bread_crumb, 1, amount_to_change);
			}
			vel_object.linear.x = 0.0;
			vel_object.angular.z = 0.0;

			ROS_INFO("odom = x: %f, y: %f, heading: %f", last_odom.pose.pose.position.x, last_odom.pose.pose.position.y, tf::getYaw(last_odom.pose.pose.orientation));
			ROS_INFO("map pose = x: %f, y: %f, heading: %f", last_map_pose.pose.position.x, last_map_pose.pose.position.y, tf::getYaw(last_map_pose.pose.orientation));

			desired_pose.header.stamp = current_time;
			desired_pose.header.frame_id = "map";
			des_pose_pub.publish(desired_pose);
		}
		
		if(stage == 1 || stage == 3  || stage == 5)
		{
			amount_to_change = goDistance(&(vel_object.linear.x), &(vel_object.angular.z), &bread_crumb, amount_to_change, REFRESH_RATE);
			updateDesiredPose(&desired_pose, &last_desired_pose, &bread_crumb, (vel_object.linear.x));
			updateBreadCrumb(&desired_pose, &last_desired_pose, &bread_crumb, (vel_object.linear.x));
		} else if(stage == 2 || stage == 4) 
		{
			amount_to_change = goRotate(&(vel_object.angular.z),amount_to_change, REFRESH_RATE);
		}

		if(stage < 6) 
		{
			// send out new command appropriate for this instant;
			// boring--always send out the same speed/spin values in this example
			cout<<"velobject = "<<vel_object.linear.x<<","<<vel_object.angular.z<<"\n";
			pub.publish(vel_object);  // this action causes the commands in vel_object to be published 
		}
		else
		{
			// done with 3 seconds of commands; now send out zeros indefinitely to halt the robot
			vel_object.linear.x = 0.0;
			vel_object.angular.z = 0.0;
			pub.publish(vel_object);
		}
    cout << "sleeping" << endl;
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (100ms) period
		//thus enforcing that we achieve the desired update rate (10Hz)
	}
	return 0; // this code will only get here if this node was told to shut down, which is
	// reflected in ros::ok() is false 
}
