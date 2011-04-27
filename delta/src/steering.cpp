#include<ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <eecs376_msgs/CrawlerDesiredState.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_listener.h>
#include "opencv2/core/core.hpp"
#include "cvFuncs.h"

using namespace std;

double LOOP_RATE = 20;	//loop rate in Hz
double kS = -0.5;		//proportional gain on following error correction
double kD = -.25;		//proportional gain on lateral error correction
double kP = -.25;		//proportional gain on heading error correction

double maxSteerV = .3;	//saturation limit for control response on linear velocity
double maxSteerW = .5;	//saturation limit for control response on angular velocity

bool stalePos = true,		//flag asserted to indicate pose hasn't updated since last steering
     staleDes = true;		//flag asserted to indicate breadcrumb hasn't updated since last steering


geometry_msgs::PoseStamped poseActual;
eecs376_msgs::CrawlerDesiredState desired;
//nav_msgs::Odometry last_odom;
tf::TransformListener *tfl;

//geometry_msgs::PoseStamped temp;
void PSOCallback(const geometry_msgs::PoseStamped::ConstPtr& poseTemp) 
{	
		poseActual.pose.position = poseTemp->pose.position;
		poseActual.pose.orientation = poseTemp->pose.orientation;
		poseActual.header = poseTemp->header;
		stalePos = false;
}

void speedCallback(const eecs376_msgs::CrawlerDesiredState::ConstPtr& newSpeed) 
{
//	cout<< "steering speed callback happened"<<endl;
        desired.header = newSpeed->header;
	desired.des_pose = newSpeed->des_pose;
	desired.des_speed = newSpeed->des_speed;
	desired.des_rho = newSpeed->des_rho;
	desired.des_lseg = newSpeed->des_lseg;
	desired.seg_type = newSpeed->seg_type;
	desired.seg_number = newSpeed->seg_number;
	staleDes = false;

	cout << "\nSPEED CALLBACK: seg_type=" << (int)desired.seg_type << ", seg_num=" << desired.seg_number << ", pose="<< desired.des_pose;
}
//Forces angles to be in range
inline double coerceAngle(double angle){
	if (angle<-CV_PI) {return angle + 2*CV_PI;}
	if (angle> CV_PI) {return angle - 2*CV_PI;}
	return angle;
}
inline double min(double a, double b){
	return (a < b)? a:b;
}
inline double max(double a, double b){
	return (a > b)? a:b;
}
inline double sign(double a){
	if (a==0)
		return 0;
	return (a>0? 1:-1);
}
/*
calculates steering errors based on the error vector (vector from desired pose to actual pose)
Following error is calculated by projecting the error vector onto a unit vector in the direction of the desired heading
Lateral error is calculated by projecting the error vector onto the leftward normal unit vector to the direction of the desired heading
Heading error is calculated by subtracting the desired heading from the actual heading 
*/
cv::Vec3f calculateSteeringParameters(geometry_msgs::Pose& poseA, geometry_msgs::Pose& poseD)
{
	cv::Vec2f posA,posD,dirA,dirD,dirN;
	ROS2CVPose(poseA,posA,dirA);
	ROS2CVPose(poseD,posD,dirD);
	getUnitVec(dirN,angle(dirD)+CV_PI/2);
	
	return cv::Vec3f((posA - posD).dot(dirD),(posA-posD).dot(dirN),coerceAngle(angle(dirA) - angle(dirD)));
}
/*
calculates steering adjustments to apply to nominal speed based on steering parameters.
For a line, the linear velocity is adjusted proportionally to the following error and the angular velocity is adjusted proportionally to both the lateral and heading errors
For an arc, the linear velocity is adjusted proportionally to the following error and the angular velocity is adjusted proportionally to both the lateral and heading errors with additional feedforward compensation for the linear velocity correction
For a corner, the angular velocity is adjusted proportionally to the following error

In the case of an invalid segment type, no adjustment is applied.
For all cases, the absolute value of the linear velocity correction is constrained by maxSteerV and the absolute value of the angular velocity correction is constrained by maxSteerW.
*/
cv::Vec2f calculateSteeringCorrections(cv::Vec3f sdp,eecs376_msgs::CrawlerDesiredState& goal){
	cv::Vec2f vw(0,0);
	switch(goal.seg_type){
			case 1:	//line
			{
				//cout<<"STEERING IN A LINE\n";
				cv::Vec3f v(kS, 0 , 0);
				cv::Vec3f w(0 , kD, kP);

				setVec(vw,sdp.dot(v),sdp.dot(w));
				break;
			}

			case 2:	//arc
			{ 
				//cout<<"STEERING IN A ARC"<<endl;
				cv::Vec3f v(kS, 0 , 0);
				cv::Vec3f w(kS * goal.des_rho , kD, kP);
					//temporarily disabled
				//setVec(vw,sdp.dot(v),sdp.dot(w));

				break;
			}

			case 3:	//turn in place
			{
				//cout<<"STEERING IN A CIRCLE"<<endl;	
				cv::Vec3f v(0,0,0);
				cv::Vec3f w(0,0,kS);
				
				setVec(vw,sdp.dot(v),sdp.dot(w));
				break;
			}

			default:
				//cout<<"STEERING IN A FAIL"<<endl;
				vw[0] = 0;
				vw[1] = 0;
		}
	vw[0] = min(maxSteerV,fabs(vw[0])) * sign(vw[0]);
	vw[1] = min(maxSteerW,fabs(vw[1])) * sign(vw[1]);
	return vw;
}
/*
calculates the nominal velocities from the NominalVelocity.
For a line, the nominal linear velocity is des_speed and the nominal angular velocity is 0.
For an arc, the nominal linear velocity is des_speed and the nominal angular velocity is des_speed * des_rho (v=wr)
For a corner, the nominal linear velocity is 0 and the nominal angular velocity is des_speed, with the sign of des_rho.
For invalid segments, the nominal velocities are both 0.
*/
cv::Vec2f getNominalVelocities(eecs376_msgs::CrawlerDesiredState& goal)
{
	switch(goal.seg_type){
		case 1:	//line
			return cv::Vec2f(goal.des_speed,0);
		case 2:	//arc
			return cv::Vec2f(goal.des_speed,goal.des_speed * goal.des_rho);
		case 3:	//turn
			return cv::Vec2f(0,goal.des_speed * sign(goal.des_rho));
		default:
			return cv::Vec2d(0,0);
	}
}
//publishes cmd_vel every time a steering correction is available at the specified loop rate
//constrains linear velocity to be non-negative to prevent backing into unviewable space
int main(int argc,char **argv)
{

	ros::init(argc,argv,"steering");//name of this node
	ros::NodeHandle n;

	// Load parameters from server
	if (n.getParam("/steering/LOOP_RATE", LOOP_RATE)){
		ROS_INFO("Steering: loaded LOOP_RATE=%f",LOOP_RATE);
	} else{
		ROS_INFO("Steering: error loading LOOP_RATE");
	}
	if (n.getParam("/steering/kD", kD)){
		ROS_INFO("Steering: loaded kD=%f",kD);
	} else{
		ROS_INFO("Steering: error loading kD");
	}
	if (n.getParam("/steering/kS", kS)){
		ROS_INFO("Steering: loaded kS=%f",kS);
	} else{
		ROS_INFO("Steering: error loading kS");
	}
	if (n.getParam("/steering/kP", kP)){
		ROS_INFO("Steering: loaded kP=%f",kP);
	} else{
		ROS_INFO("Steering: error loading kP");
	}
	if (n.getParam("/steering/maxSteerV", maxSteerV)){
		ROS_INFO("Steering: loaded maxSteerV=%f",maxSteerV);
	} else{
		ROS_INFO("Steering: error loading maxSteerV");
	}
	if (n.getParam("/steering/maxSteerW", maxSteerW)){
		ROS_INFO("Steering: loaded maxSteerW=%f",maxSteerW);
	} else{
		ROS_INFO("Steering: error loading maxSteerW");
	}


	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	tfl = new tf::TransformListener();
	ros::Subscriber sub1 = n.subscribe<geometry_msgs::PoseStamped>("poseActual", 1, PSOCallback);
	ros::Subscriber sub2 = n.subscribe<eecs376_msgs::CrawlerDesiredState>("NominalSpeed", 1, speedCallback);

	geometry_msgs::Twist vel_object;
	ros::Duration elapsed_time; // define a variable to hold elapsed time
	ros::Rate loopTimer(LOOP_RATE); //will perform sleeps to enforce loop rate of "10" Hz
	cout<<"Starting steering"<<endl;
	while (!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly. Wait until ros::Time::now()
	ros::Time birthday = ros::Time::now();

	cv::Vec3f sdp;
	cv::Vec2d vw;

	while(ros::ok()){
		loopTimer.sleep();
		ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely

		ros::Time current_time = ros::Time::now();
	 	desired.header.stamp = current_time;
		elapsed_time= ros::Time::now()-birthday;
		//ROS_INFO("birthday is %f", birthday.toSec());
		//ROS_INFO("elapsed time is %f", elapsed_time.toSec());	
		if(stalePos || staleDes)	{
			cout << "stale\n";
		}
		else{
			//cout<<"Steering activate! "<<(int)desired.seg_type<<"  "<<desired.seg_number<<endl;
		
			sdp = calculateSteeringParameters(poseActual.pose, desired.des_pose);
			ROS_INFO("\tactual:      %f,%f,%f", poseActual.pose.position.x, poseActual.pose.position.y, tf::getYaw(poseActual.pose.orientation));
			ROS_INFO("\tdesired:     %f,%f,%f", desired.des_pose.position.x, desired.des_pose.position.y, tf::getYaw(desired.des_pose.orientation));
			ROS_INFO("\tserrors:                   %f,%f,%f ",sdp[0],sdp[1],sdp[2]);		
			vw = getNominalVelocities(desired);
			bool requestReverse = vw[0] < 0;
			vw += calculateSteeringCorrections(sdp,desired);

			if(!requestReverse){	//only allow reverse if desired speed < 0		
				vw[0]  = max(vw[0], 0);
			}
			
			vel_object.linear.x = vw[0];
			vel_object.angular.z= vw[1];
			stalePos = true,
				staleDes = true;

			cout<<"steering:\n\tNominalSpeed "<<desired.des_speed<<"\n\tcommanded "<<vw[0]<<" , "<<vw[1]<<endl;
		}
		pub.publish(vel_object);
	}
}
