#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <opencv/cv.h>

using namespace cv;
using namespace geometry_msgs;

// Two state vectors: one only from open loop odometry, and the other from both odometry and GPS.
Mat state_odom_only;
Mat state_inc_GPS;

// TODO: Get correct value for track width
double TRACK_WIDTH = 0.5;
// The robot travels this far in between each virtual heading update
double DIST_BETWEEN_HEADING_UPDATES = 1.0;
double LOOP_RATE = 10;

// Initializes data
void initFilters()
{
	// Initialize the state vectors to 3x1 floating point
	state_odom_only = Mat::zeros(3, 1, CV_32F);
	state_inc_GPS = Mat::zeros(3, 1, CV_32F);
	
}

void GPSCallback(/* TODO: arguments */)
{
	// Calculate the weighting factor for the filter
	
	// Apply the filter to state_inc_GPS
}

void odomCallback(/*TODO: arguments */)
{
	double s_right, s_left;
	
	// TODO: Calculate s_right and s_left, the wheel movements in meters, from odometry.
	
	// Update the state of the robot with the latest odometry
	//state_odom_only = updateState(state_odom_only, s_right, s_left);
	//state_inc_GPS = updateState(state_inc_GPS, s_right, s_left);
	
	// Check if the robot has gone over DIST_BETWEEN_HEADING_UPDATES.  If so, apply heading correction.
}

// Given  a state [x;y;psi] and wheel movements in meters (s_right and s_left) returns a state updated according to the linear system x(k+1) = A*x(k) + B*u(k)
Mat updateState(Mat state, float s_right, float s_left)
{
	// For convenience, grab the current angle
	double psi = state.at<float>(2,0);
	
	// Generate the control matrix B
	float B_temp[3][2] = {{0.5*cos(psi),0.5*cos(psi)},{0.5*sin(psi),0.5*cos(psi)},{1.0/TRACK_WIDTH,-1.0/TRACK_WIDTH}};
	Mat B = Mat(3, 2, CV_32F, B_temp);
	
	// Generate input vector u
	float u_temp[2][1] = {{s_right},{s_left}};
	Mat u = Mat(2, 1, CV_32F, u_temp);
	
	// Update the linear system according to the control law (note that A is the identity matrix in this case)
	Mat new_state = state + B*u;
	
	return(new_state);
}

// Returns the latest pose estimate incorporating both odometry and GPS
geometry_msgs::Pose getPositionEstimate()
{
	Mat state = Mat(state_inc_GPS);
	
	// Load the state from the matrix, and return it as a pose
	geometry_msgs::Pose p;
	p.position.x = state.at<float>(0,0);
	p.position.y = state.at<float>(1,0);
	p.orientation = tf::createQuaternionMsgFromYaw(state.at<float>(2,0));
	return(p);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pso");
	ros::NodeHandle n;
	ROS_INFO("pso initialized");
	
	// TODO: Create subscribers for GPS and odom, create publisher for position
	
	// Wait for ROS to start	
	while (!ros::ok()){ ros::spinOnce(); }
	


	ros::Rate loopTimer(LOOP_RATE);
	while(ros::ok())
	{
		ros::spinOnce();
		
		// TODO: Publish the pose returned by getPositionEstimate
		
		loopTimer.sleep();
	}
	return 0;
}
