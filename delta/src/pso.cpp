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
#include <geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include <iostream>
using namespace std;
using namespace cv;
using namespace geometry_msgs;
using namespace nav_msgs;
class LinFit
{
	public:
	//the various sums we need to keep
	double x;
	double xy;
	double y;
	double x2;
	int n;

	LinFit();
	void next(double x, double y);
	double getSlope();
	double getHeading();
};

LinFit::LinFit()
{
	n = x = xy = y = x2 = 0;
}

void LinFit::next(double x, double y)
{
	this->x += x;
	this->y += y;
	this->xy += x*y;
	this->x2 += x*x;
	this->n++;
}

double LinFit::getSlope()
{
	if(n>0)
	{
		return (xy-x*y/n)/(x2-x*x/n);
	}
	else
	{
		return 0;
	}
}

double LinFit::getHeading()
{
	return atan2(getSlope(), 1.0);
}


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
double HEADING_WEIGHT = 0.2;

ros::Publisher pose_pub;
ros::Subscriber gps_sub;
ros::Subscriber odom_sub;
ros::Publisher cmdvel_pub;

bool orient = true;
//true = working on orientation.
//false = doing normal data publishing

void PrintMat(CvMat *A, FILE* f=stdout)
{
    int i, j;
    for (i = 0; i < A->rows; i++)
    {
        fprintf(f,"\n");
        switch (CV_MAT_DEPTH(A->type))
        {
            case CV_32F:
            case CV_64F:
                for (j = 0; j < A->cols; j++)
                fprintf (f,"%8.6f ", (float)cvGetReal2D(A, i, j));
                break;
            case CV_8U:
            case CV_16U:
                for(j = 0; j < A->cols; j++)
                fprintf (f,"%6d",(int)cvGetReal2D(A, i, j));
                break;
            default:
                break;
        }
    }
    fprintf(f,"\n");
}
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
	cerr<<"heading adjustment: "<<(theta_gps - theta_squiggle)<<endl;
	state_estimate[2] += HEADING_WEIGHT * (theta_gps - theta_squiggle);
	if(state_estimate[2] > CV_PI) {
		state_estimate[2] -= 2 * CV_PI;
	} else if(state_estimate[2] < -CV_PI) {
		state_estimate[2] += 2 * CV_PI;		
	}
	//cout<<"state_estimate 2 = "<<state_estimate[2]<<", "<<HEADING_WEIGHT*(theta_gps-theta_squiggle)<<"\n";
	state_last_fix = state_estimate;
	state_odom_only = state_estimate;
}


void gpsUpdateState(Vec3f gpsFix, float a){
	gpsFix[2] =  state_inc_GPS[2];	//hack to preserve heading
	Vec3f bestGuess = a * state_inc_GPS + (1-a) * gpsFix;

	double x = (bestGuess[0]-state_last_fix[0]);
	double y = (bestGuess[1]-state_last_fix[1]);

	if(sqrt(x*x+y*y)>=DIST_BETWEEN_HEADING_UPDATES && a<1){
		cerr<<"mangling heading\n";
		applyCorrection(bestGuess,gpsFix);
	}
	state_inc_GPS = bestGuess;
	cout<<"(x,y,h) "<<bestGuess[0]<<","<<bestGuess[1]<<","<<bestGuess[2]<<endl;
}

Vec3f gpsToReasonableCoords(cwru_base::NavSatFix gps_world_coords) {
	double x = (gps_world_coords.latitude - 41.5);
	double y = (gps_world_coords.longitude + 81.605); 
	Vec3f coords(x * 111090.0 - 217.168,
		     -1 * (y * 81968.0 + 149.419),
		     0);
	return coords;
}

LinFit fitter;
bool getposition = false;
Vec3f pos;
float poscount = 0.0;
void GPSCallback(const cwru_base::NavSatFix::ConstPtr& gps_world_coords)
{
	//based on the comments in the cwru_base stuff, a status of -1 is a bad fix, and statuses >=0 represent valid coordinates
	bool goodCoords = (gps_world_coords->status.status>0);

	Vec3f gpsAllegedState = gpsToReasonableCoords(*gps_world_coords); //should convert to a reasonable mat in meters
	if(orient){
		fitter.next(gpsAllegedState[0],gpsAllegedState[1]);
		if(getposition){
				pos+= gpsAllegedState;
				poscount+=1.0;
		}
		return;
	}
	// Calculate the weighting factor for the filter
	//newman said this might be good in class.  
	//If it isn't, use gps_world_coords.position_covariance, which should be a 9 element 1D array.
	float alpha = goodCoords? 0.9:1;
	//cerr << gpsAllegedState[0] << "," << gpsAllegedState[1] << "," << gpsAllegedState[2] << endl;	
	// Apply the filter to state_inc_GPS
	gpsUpdateState(gpsAllegedState, alpha);
	
}

// Given  a state [x;y;psi] and wheel movements in meters (s_right and s_left) returns a state updated according to the linear system x(k+1) = A*x(k) + B*u(k)
Vec3f odomUpdateState(Vec3f state, float s_right, float s_left)
{
	// For convenience, grab the current angle
	double psi = state[2];
	CvMat stateOld = Mat(state);
	//PrintMat(&stateOld);	
	// Generate the control matrix B
	Mat B = (Mat_<float>(3,2) << 0.5*cos(psi),0.5*cos(psi),0.5*sin(psi),0.5*sin(psi),1.0/TRACK_WIDTH,-1.0/TRACK_WIDTH);
	
	// Generate input vector u
	Mat u = (Mat_<float>(2,1) << s_right,s_left);
	// Update the linear system according to the control law (note that A is the identity matrix in this case)
	Mat_<float> new_state = Mat(state) + B*u;
	
	Vec3f newstate(new_state(0),new_state(1),new_state(2));
	return(newstate);
}

geometry_msgs::PoseStamped getPositionEstimate();
int rolex = 0;//because clock is already taken
#define ALARM_CLOCK 50*60
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
	if(orient) {
		return; //don't publish while we're orienting
	}
	//if(rolex<ALARM_CLOCK)rolex++;
	//else
	{
		// the final output should have this type and call the publish function on pose_pub
		nav_msgs::Odometry odom = stateToOdom(state_inc_GPS); 
		//see http://www.ros.org/doc/api/nav_msgs/html/msg/Odometry.html for the stuff that it has.
		pose_pub.publish(getPositionEstimate());
	}
	
}

// Returns the latest pose estimate incorporating both odometry and GPS
geometry_msgs::PoseStamped getPositionEstimate()
{
	Mat state = Mat(state_inc_GPS);
	
	// Load the state from the matrix, and return it as a pose
	geometry_msgs::PoseStamped p;
	p.pose.position.x = state_inc_GPS[0];
	p.pose.position.y = state_inc_GPS[1];
	if(state_inc_GPS[2] > CV_PI) {
		state_inc_GPS[2] -= 2 * CV_PI;
	} else if(state_inc_GPS[2] < -1 * CV_PI) {
		state_inc_GPS[2] += 2 * CV_PI;
	}
	p.pose.orientation = tf::createQuaternionMsgFromYaw(state_inc_GPS[2]);
	return(p);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pso");
 
        //while(!ros::Time::isValid()) { ros::spinOnce(); }
     
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
	cmdvel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	int debug_ctr = 0;
	geometry_msgs::PoseStamped temp_pose;

	ros::Rate loopTimer(LOOP_RATE);
	while(ros::ok())
	{
		ros::spinOnce();
	/*	
		// Publish the latest pose estimate
		temp_pose = getPositionEstimate();
		pose_pub.publish(temp_pose);
	*/
	//get our initial haeading
	
		if(orient) {
			static int wait_count = 0;
			static double currentspeed = 0.0;
			static bool firsttime = true;
			static bool waiting = true;
			if(waiting) {
				wait_count++;
				currentspeed = 0.0;
				if(wait_count > 10 * LOOP_RATE) {
					ROS_INFO("moving: %f", LOOP_RATE);
					
					waiting = false;
					wait_count = 0;
					if(!firsttime) {
						orient = false;
						cmdvel_pub.shutdown();
						pos = pos * (1/poscount);
						float heading = (float)fitter.getHeading();
						pos[2] = heading;
						state_odom_only = pos;
						state_inc_GPS = pos;
						state_last_fix = pos;
						getposition = false;
						continue;
					}
				}
			} else {
				currentspeed = 0.1;
				wait_count++;
				if(wait_count > 25 * LOOP_RATE) {
					wait_count = 0;
					firsttime = false;
					waiting = true;
					getposition = true;
				}
			
			}
			ROS_INFO("publishing %f",currentspeed);
			Twist vel_object;
			vel_object.linear.x = currentspeed; 
			vel_object.angular.z = 0.0;
			cmdvel_pub.publish(vel_object);			
		}
		// Every so often, spit out info for debugging
		if( debug_ctr++ % 20 == 0 )
		{
			//ROS_INFO("PSO: at (%f,%f), psi=%f",temp_pose.pose.position.x,temp_pose.pose.position.y,tf::getYaw(temp_pose.pose.orientation));
		}
		
		loopTimer.sleep();
	}
	return 0;
}
