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

using namespace cv;
using namespace geometry_msgs;

// Two state vectors: one only from open loop odometry, and the other from both odometry and GPS.
Vec3f state_odom_only;
Vec3f state_inc_GPS;
Vec3f state_last_fix;	//best-guess state at last heading update

// TODO: Get correct value for track width
double TRACK_WIDTH = 0.56515;

// The robot travels this far in between each virtual heading update
double DIST_BETWEEN_HEADING_UPDATES = 1.0;

// Rate (in Hz) of main loop
double LOOP_RATE = 10;

// Trust the virtual heading sensor this much
double HEADING_WEIGHT = 0.5;

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

nav_msgs::Odometry stateToOdom(Vec3f gpsState){
	nav_msgs::Odometry odomState;
	odomState.pose.pose.position.x = gpsState[0];
	odomState.pose.pose.position.y = gpsState[1];
	odomState.pose.pose.orientation = tf::createQuaternionMsgFromYaw(gpsState[2]);
	odomState.header.frame_id = "map";
	return odomState;
}

void applyCorrection(Vec3f& state_estimate, Vec3f gps_fix){
	double theta_squiggle = atan2(state_estimate[1] - state_last_fix[1],state_estimate[0] - state_last_fix[0]);
	double theta_gps = atan2(gps_fix[1] - state_last_fix[1],gps_fix[0]-state_last_fix[0]);
	state_estimate[2] += HEADING_WEIGHT * (theta_gps - theta_squiggle);
	state_last_fix = state_estimate;
	state_odom_only = state_estimate;
}


void gpsUpdateState(Vec3f gpsFix, float a){
	gpsFix[2] = (1-a) * state_inc_GPS[2];	//hack to preserve heading
	Vec3f bestGuess = a * state_inc_GPS + (1-a) * gpsFix;

	double x = (bestGuess[0]-state_last_fix[0]);
	double y = (bestGuess[1]-state_last_fix[1]);

	if(sqrt(x*x+y*y)>=DIST_BETWEEN_HEADING_UPDATES && a<1){
		applyCorrection(bestGuess,gpsFix);
	}
	state_inc_GPS = bestGuess;
}

Vec3f gpsToReasonableCoords(cwru_base::NavSatFix gps_world_coords) {
	double x = (gps_world_coords.latitude - 41.5);
	double y = (gps_world_coords.longitude + 81.605); 
	Vec3f coords(x * 111090.0,
		     y * 81968.0,
		     0);
	return coords;
} 
void GPSCallback(const cwru_base::NavSatFix::ConstPtr& gps_world_coords)
{

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
	static int s_right_prev = cRIO->right_wheel_encoder;
	static int s_left_prev = cRIO->left_wheel_encoder;
	float ds_left = (cRIO->left_wheel_encoder - s_left_prev) * 0.0006656;	//into meters
	float ds_right = (cRIO->right_wheel_encoder - s_right_prev)* 0.00070311;//into meters

	s_right_prev = cRIO->right_wheel_encoder;
	s_left_prev  = cRIO->left_wheel_encoder;
	

	// TODO: Convert from encoder ticks to meters
	
	// Update the state of the robot with the latest odometry
	//be aware that these are all ints and need to be converted to sensible units before use.
	state_odom_only = odomUpdateState(state_odom_only, ds_right, ds_left);
	state_inc_GPS = odomUpdateState(state_inc_GPS, ds_right, ds_left);

	// the final output should have this type and call the publish function on pose_pub
	nav_msgs::Odometry odom = stateToOdom(state_inc_GPS); 
	//see http://www.ros.org/doc/api/nav_msgs/html/msg/Odometry.html for the stuff that it has.
	pose_pub.publish(odom);
}

// Returns the latest pose estimate incorporating both odometry and GPS
geometry_msgs::PoseStamped getPositionEstimate()
{
	Mat state = Mat(state_inc_GPS);
	
	// Load the state from the matrix, and return it as a pose
	geometry_msgs::PoseStamped p;
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
	
	// Load parameters from server
	if (n.getParam("/pso/heading_weight", HEADING_WEIGHT)){
		ROS_INFO("PSO: loaded heading_weight=%f", HEADING_WEIGHT);
	} else{
		ROS_INFO("PSO: error loading heading_weight");
	}
	
	if (n.getParam("/pso/loop_rate", LOOP_RATE)){ //Frequency of main loop
		ROS_INFO("PSO: loaded loop_rate=%f", LOOP_RATE);
	} else{
		ROS_INFO("PSO: error loading loop_rate");
	}
	
	if (n.getParam("/pso/dist_threshold", DIST_BETWEEN_HEADING_UPDATES)){ //Frequency of main loop
		ROS_INFO("PSO: loaded dist_threshold=%f", DIST_BETWEEN_HEADING_UPDATES);
	} else{
		ROS_INFO("PSO: error loading dist_threshold");
	}
	
	// Wait for ROS to start	
	while (!ros::ok()){ ros::spinOnce(); }
	
	gps_sub = n.subscribe<cwru_base::NavSatFix>("gps_fix", 1, GPSCallback);
	odom_sub = n.subscribe<cwru_base::cRIOSensors>("crio_sensors", 1, odomCallback);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("poseActual", 1);

	int debug_ctr = 0;
	geometry_msgs::PoseStamped temp_pose;

	ros::Rate loopTimer(LOOP_RATE);
	while(ros::ok())
	{
		ros::spinOnce();
		
		// Publish the latest pose estimate
		temp_pose = getPositionEstimate();
		pose_pub.publish(temp_pose);
		
		// Every so often, spit out info for debugging
		if( debug_ctr++ % 20 == 0 )
		{
			ROS_INFO("PSO: at (%f,%f), psi=%f",temp_pose.position.x,temp_pose.position.y,tf::getYaw(temp_pose.orientation));
		}
		
		loopTimer.sleep();
	}
	return 0;
}
