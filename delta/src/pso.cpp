#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <opencv/cv.h>
#include <cwru_base/NavSatFix.h>
#include <cwru_base/NavSatStatus.h>
#include <cwru_base/cRIOSensors.h>
#include<nav_msgs/Odometry.h>



#define DIST_THRESHOLD 1//meter

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

ros::Publisher pose_pub;
ros::Subscriber gps_sub;
ros::Subscriber odom_sub;

// Initializes data
void initFilters()
{
	// Initialize the state vectors to 3x1 floating point
	state_odom_only = Mat::zeros(3, 1, CV_32F);
	state_inc_GPS = Mat::zeros(3, 1, CV_32F);
	
}

void gpsUpdateState(Mat& state, Mat gpsAllegedState, float alpha)
{
	float heading = state.at<float>(2);
	state = alpha*state+(1-alpha)*gpsAllegedState;
	state.at<float>(2) = heading;
}

void GPSCallback(const cwru_base::NavSatFix::ConstPtr& gps_world_coords)
{
	//based on the comments in the cwru_base stuff, a status of -1 is a bad fix, and statuses >=0 represent valid coordinates
	bool goodCoords;
	if(gps_world_coords.status.status>0) goodCoords = true;
	else goodCoords = false;

	Mat gpsAllegedState = gpsToReasonableCoords(gps_world_coords); //should convert to a reasonable mat in meters

	// Calculate the weighting factor for the filter
	//newman said this might be good in class.  
	//If it isn't, use gps_world_coords.position_covariance, which should be a 9 element 1D array.
	float alpha;
	if(goodCoords) alpha = .1; 
	else alpha = 1;
	
	// Apply the filter to state_inc_GPS
	gpsUpdateState(state_inc_GPS, gpsAllegedState, alpha);
	
}

// Given  a state [x;y;psi] and wheel movements in meters (s_right and s_left) returns a state updated according to the linear system x(k+1) = A*x(k) + B*u(k)
Mat odomUpdateState(Mat state, float s_right, float s_left)
{
	// For convenience, grab the current angle
	double psi = state.at<float>(2,0);
	
	// Generate the control matrix B
	float B_temp[3][2];//{{0.5*cos(psi),0.5*cos(psi)},{0.5*sin(psi),0.5*cos(psi)},{1.0/TRACK_WIDTH,-1.0/TRACK_WIDTH}};
	B_temp[0,0] = 0.5*cos(psi);//are you sure this is right? I didn't pay attention in class, but the 3 cosines of psi make me think it is wrong.  -wes
	B_temp[0,1] = 0.5*cos(psi);
	B_temp[1,0] = 0.5*sin(psi);
	B_temp[1,1] = 0.5*cos(psi);
	B_temp[2,0] = 1.0/TRACK_WIDTH;
	B_temp[2,1] = -1.0/TRACK_WIDTH;
	Mat B = Mat(3, 2, CV_32F, B_temp);
	
	// Generate input vector u
	float u_temp[2][1];// = {{s_right},{s_left}};
	u_temp[0][0] = s_right;
	u_temp[1][0] = s_left;
	Mat u = Mat(2, 1, CV_32F, u_temp);
	
	// Update the linear system according to the control law (note that A is the identity matrix in this case)
	Mat new_state = state + B*u;
	
	return(new_state);
}

void odomCallback(const cwru_base::cRIOSensors::ConstPtr& cRIO)
{
	//see the cRIOSensors.msg in cwru_semi_stable
	int s_right = cRIO->right_wheel_encoder;
	int s_left = cRIO->left_wheel_encoder

	// Update the state of the robot with the latest odometry
	//be aware that these are all ints and need to be converted to sensible units before use.
	state_odom_only = odomUpdateState(state_odom_only, s_right, s_left);
	state_inc_GPS = odomUpdateState(state_inc_GPS, s_right, s_left);
	
	// Check if the robot has gone over DIST_BETWEEN_HEADING_UPDATES.  If so, apply heading correction.
	double x = state_odom_only.at<float>(0,0)-state_inc_GPS.at<float>(0,0);
	double y = state_odom_only.at<float>(1,0)-state_inc_GPS.at<float>(1,0);
	double dist = sqrt(x*x+y*y);

	if(dist>DIST_THRESHOLD) applyCorrection(state_odom_only, state_inc_GPS);

	// the final output should have this type and call the publish function on pose_pub
	nav_msgs::odometry odom = stateToOdom(state_inc_gps); 
	//see http://www.ros.org/doc/api/nav_msgs/html/msg/Odometry.html for the stuff that it has.
	pose_pub.publish(odom);
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
	
	gps_sub = n.subscribe<cwru_base::NavSatFix>("gps_fix", 1, GPSCallback);
	odom_sub = n.subscribe<cwru_base::cRIOSensors>("crio_sensors", 1, odomCallback);
	pose_pub = n.advertise<nav_msgs::Odometry>("odom", 1);

	ros::Rate loopTimer(LOOP_RATE);
	while(ros::ok())
	{
		ros::spinOnce();
		
		// TODO: Publish the pose returned by getPositionEstimate
		
		loopTimer.sleep();
	}
	return 0;
}
