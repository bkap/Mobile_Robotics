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
Vec3f state_odom_only;
Vec3f state_inc_GPS;
Vec3f state_last_fix;	//best-guess state at last heading update
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
	state_odom_only = Vec3f(0,0,0);
	state_inc_GPS = Vec3f(0,0,0);
	state_last_fix = Vec3f(0,0,0);
}

void applyCorrection(Vec3f& state_estimate, Vec3f gps_fix){
	double theta_squiggle = atan2(state_estimate[1] - state_last_fix[1],state_estimate[0] - state_last_fix[0]);
	double theta_gps = atan2(gps_fix[1] - state_last_fix[1],gps_fix[0]-state_last_fix[0]);
	state_estimate[2] += 0.5 * (theta_gps - theta_squiggle);
	state_last_fix = state_estimate;
	state_odom_only = state_estimate;
}


void gpsUpdateState(Vec3f gpsFix, float a){
	gpsFix[2] = (1-a) * state_inc_GPS[2];	//hack to preserve heading
	Vec3f bestGuess = a * state_inc_GPS + (1-a) * gpsFix;
	if(norm(bestGuess-state_last_fix)>=DIST_BETWEEN_HEADING_UPDATES && a<1){
		applyCorrection(bestGuess,gpsFix);
	}
	state_inc_GPS = bestGuess;
}

Vec3f gpsToReasonableCoords(cwru_base::NavSatFix gps_world_coords) {
	double x_rad = CV_PI / 180. * (gps_world_coords.latitude - 41.5);
	double y_rad = CV_PI / 180. * (gps_world_coords.longitude + 81.605); 
	Vec3f coords;
	coords[0] = (float)(x_rad * 6378100);
	coords[1] = (float)(y_rad * 6378100);
	coords[2] = 0;
	return coords;
}
void GPSCallback(const cwru_base::NavSatFix::ConstPtr& gps_world_coords)
{
	double s_right, s_left;
	
	// TODO: Calculate s_right and s_left, the wheel movements in meters, from odometry.
	
	// Update the state of the robot with the latest odometry
	//state_odom_only = updateState(state_odom_only, s_right, s_left);
	//state_inc_GPS = updateState(state_inc_GPS, s_right, s_left);

	//based on the comments in the cwru_base stuff, a status of -1 is a bad fix, and statuses >=0 represent valid coordinates
	bool goodCoords = (gps_world_coords->status.status>0);

	Vec3f gpsAllegedState = gpsToReasonableCoords(*gps_world_coords); //should convert to a reasonable mat in meters

	// Calculate the weighting factor for the filter
	//newman said this might be good in class.  
	//If it isn't, use gps_world_coords.position_covariance, which should be a 9 element 1D array.
	float alpha = goodCoords? 0.9:1;
	
	// Apply the filter to state_inc_GPS
	gpsUpdateState(gpsAllegedState, alpha);
	
}

// Given  a state [x;y;psi] and wheel movements in meters (s_right and s_left) returns a state updated according to the linear system x(k+1) = A*x(k) + B*u(k)
Vec3f odomUpdateState(Vec3f state, float s_right, float s_left)
{
	// For convenience, grab the current angle
	double psi = state[2];
	
	// Generate the control matrix B
	Mat B = (Mat_<float>(3,2) << 0.5*cos(psi),0.5*cos(psi),0.5*sin(psi),0.5*sin(psi),1.0/TRACK_WIDTH,-1.0/TRACK_WIDTH);
	
	// Generate input vector u
	Mat u = (Mat_<float>(2,1) << s_right,s_left);
	// Update the linear system according to the control law (note that A is the identity matrix in this case)
	Mat_<float> new_state = Mat(state) + B*u;
	
	Vec3f newstate(new_state(0),new_state(1),new_state(2));
	return(newstate);
}

void odomCallback(const cwru_base::cRIOSensors::ConstPtr& cRIO)
{
	//see the cRIOSensors.msg in cwru_semi_stable
	int s_right = cRIO->right_wheel_encoder;
	int s_left = cRIO->left_wheel_encoder;

	// Update the state of the robot with the latest odometry
	//be aware that these are all ints and need to be converted to sensible units before use.
	state_odom_only = odomUpdateState(state_odom_only, s_right, s_left);
	state_inc_GPS = odomUpdateState(state_inc_GPS, s_right, s_left);
	
	// Check if the robot has gone over DIST_BETWEEN_HEADING_UPDATES.  If so, apply heading correction.

	// the final output should have this type and call the publish function on pose_pub
	nav_msgs::Odometry odom = stateToOdom(state_inc_GPS); 
	//see http://www.ros.org/doc/api/nav_msgs/html/msg/Odometry.html for the stuff that it has.
	pose_pub.publish(odom);
}

nav_msgs::Odometry stateToOdom(Mat gpsState){
	nav_msgs::Odometry odomState;
	odomState.pose.pose.position.x = gpsState.at<float>(0,0);
	odomState.pose.pose.position.y = gpsState.at<float>(1,0);
	odomState.pose.pose.orientation = tf::createQuaternionMsgFromYaw(gpsState.at<float>(2,0));
	return odomState;
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
