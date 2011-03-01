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

#define pi 3.14159265358979323846264338327950288


using namespace std;

const double LOOP_RATE = 10;
const double kS = -1.0;
const double kD = -.75;
const double kP = -.75;

const double maxSteerV = .3;
const double maxSteerW = .5;

bool stalePos = true,
     staleDes = true;


geometry_msgs::PoseStamped poseActual;
eecs376_msgs::CrawlerDesiredState desired;
//nav_msgs::Odometry last_odom;
tf::TransformListener *tfl;

geometry_msgs::PoseStamped temp;
void PSOCallback(const nav_msgs::Odometry::ConstPtr& odom) 
{	
//	cout<<"steering odom callback happened"<<endl;
//	last_odom = *odom;

        temp.pose = odom->pose.pose;
	temp.header = odom->header;
        //cout<<"temp "<<temp.pose.position.x<<" , "<<temp.pose.position.y<<endl;
        try 
	{
		tfl->transformPose("map", temp, poseActual);
		stalePos = false;
        } 
	catch (tf::TransformException ex) 
	{
	  cout << "We caught an error!" << endl;
          ROS_ERROR("%s", ex.what());
        }
	
	/////////cout<<"finished"<<endl;
	
}

void speedCallback(const eecs376_msgs::CrawlerDesiredState::ConstPtr& newSpeed) 
{
//	cout<< "steering speed callback happened"<<endl;
	//desired = *newSpeed;
        desired.header = newSpeed->header;
	desired.des_pose = newSpeed->des_pose;
	desired.des_speed = newSpeed->des_speed;
	desired.des_rho = newSpeed->des_rho;
	desired.des_lseg = newSpeed->des_lseg;
	desired.seg_type = newSpeed->seg_type;
	desired.seg_number = newSpeed->seg_number;
	staleDes = false;
	cout << "\nSPEED CALLBACK: seg_type=" << (int)desired.seg_type << ", seg_num=" << desired.seg_number << ", pose=", desired.des_pose;
}

inline double coerceAngle(double angle){
	if (angle<-pi) {return angle + 2*pi;}
	if (angle> pi) {return angle - 2*pi;}
	return angle;
}

inline double min(double a, double b){
	return (a < b)? a:b;
}
inline double max(double a, double b){
	return (a > b)? a:b;
}

cv::Vec3d calculateSteeringParameters(geometry_msgs::Pose *poseA, geometry_msgs::Pose *poseD)
{
	cv::Vec2d posA(poseA->position.x,poseA->position.y);
	cv::Vec2d posD(poseD->position.x,poseD->position.y);
	double dirA= tf::getYaw(poseA->orientation);
	double dirD=tf::getYaw(poseD->orientation);
	cv::Vec2d psiA(cos(dirA),sin(dirA));
	cv::Vec2d psiD(cos(dirD),sin(dirD));
	cv::Vec2d psiN(cos(dirD+pi/2),sin(dirD+pi/2));
 
	return cv::Vec3d((posA - posD).dot(psiD),(posA - posD).dot(psiN),coerceAngle(dirA - dirD));
}

cv::Vec2d calculateSteeringCorrections(cv::Vec3d sdp,eecs376_msgs::CrawlerDesiredState *goal){
	cv::Vec2d vw(0,0);
	switch(goal->seg_type){
			case 1:	//line
			{
				cout<<"STEERING IN A LINE\n";

				vw[0] = kS * sdp[0];
				vw[1] = kD*sdp[1]+kP*sdp[2];
				break;
			}

			case 2:	//arc
			{
				cout<<"STEERING IN A ARC"<<endl;

				vw[0] =  kS * sdp[0];
				vw[1] = vw[0] * goal->des_rho + kD*sdp[1] + kP*sdp[2];
				break;
			}

			case 3:	//turn in place
			{
				cout<<"STEERING IN A CIRCLE"<<endl;	

				vw[0] = 0;
				vw[1] = kS * sdp[0];
				break;
			}

			default:
				cout<<"STEERING IN A FAIL"<<endl;
				vw[0] = 0;
				vw[1] = 0;
		}
	vw[0] = (vw[0] > 0)? min(maxSteerV, vw[0]) : max(-maxSteerV, vw[0]);
	vw[1] = (vw[1] > 0)? min(maxSteerW, vw[1]) : max(-maxSteerW, vw[1]);
	return vw;
}
cv::Vec2d getNominalVelocities(eecs376_msgs::CrawlerDesiredState* goal)
{
	switch(goal->seg_type){
			case 1:	//line
				return cv::Vec2d(goal->des_speed,0);
			case 2:	//arc
				return cv::Vec2d(goal->des_speed,goal->des_speed * goal->des_rho);
			case 3:	//turn
				return cv::Vec2d(0,goal->des_speed);
			default:
				return cv::Vec2d(0,0);
	}
}
int main(int argc,char **argv)
{

	ros::init(argc,argv,"steering");//name of this node
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	tfl = new tf::TransformListener();
	ros::Subscriber sub1 = n.subscribe<nav_msgs::Odometry>("odom", 1, PSOCallback);
	ros::Subscriber sub2 = n.subscribe<eecs376_msgs::CrawlerDesiredState>("NominalSpeed", 1, speedCallback);

	geometry_msgs::Twist vel_object;
	ros::Duration elapsed_time; // define a variable to hold elapsed time
	ros::Rate loopTimer(LOOP_RATE); //will perform sleeps to enforce loop rate of "10" Hz
	cout<<"Starting steering"<<endl;
	while (!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly. Wait until ros::Time::now()
	ros::Time birthday = ros::Time::now();

	cv::Vec3d sdp;
	cv::Vec2d vw;

	while(ros::ok()){
//		cout<<"STEERING\n";
		loopTimer.sleep();
//		cout<<"STEERING AWAKE"<<endl;
		ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely

		ros::Time current_time = ros::Time::now();
	 	desired.header.stamp = current_time;
		elapsed_time= ros::Time::now()-birthday;
		//ROS_INFO("birthday is %f", birthday.toSec());
		//ROS_INFO("elapsed time is %f", elapsed_time.toSec());	
		if(stalePos)	{continue;}
		cout<<"Steering activate! "<<(int)desired.seg_type<<"  "<<desired.seg_number<<endl;
		
		sdp = calculateSteeringParameters(&poseActual.pose, &desired.des_pose);
		cout<<"\tserrors:                    "<<sdp[0]<<","<<sdp[1]<<","<<sdp[2]<<endl;		
		vw= getNominalVelocities(&desired);
		vw += calculateSteeringCorrections(sdp,&desired);
		
		vw[0]  = max(vw[0], 0);

		//	cout<<"Steering calculated"<<endl;
				//limit velocities
			
			vel_object.linear.x = vw[0];
			vel_object.angular.z= vw[1];
			stalePos = true,
		     	staleDes = true;

			cout<<"steering:\n\tNominalSpeed "<<desired.des_speed<<"\n\tcommanded "<<vw[0]<<" , "<<vw[1]<<endl;
			pub.publish(vel_object);
	}
}
