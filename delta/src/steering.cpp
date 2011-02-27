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
const double kS = .1;
const double kD = .75;
const double kP = .75;

bool stalePos = true,
     staleDes = true;


geometry_msgs::PoseStamped poseActual;
eecs376_msgs::CrawlerDesiredState desired;
nav_msgs::Odometry last_odom;
tf::TransformListener *tfl;

geometry_msgs::PoseStamped temp;
void PSOCallback(const nav_msgs::Odometry::ConstPtr& odom) 
{	
	cout<<"steering odom callback happened"<<endl;
	last_odom = *odom;

        temp.pose = last_odom.pose.pose;
	temp.header = last_odom.header;
        //cout<<"temp "<<temp.pose.position.x<<" , "<<temp.pose.position.y<<endl;
        try 
	{
          tfl->transformPose("map", temp, poseActual);
        } 
	catch (tf::TransformException ex) 
	{
	  cout << "We caught an error!" << endl;
          ROS_ERROR("%s", ex.what());
        }
	/////////cout<<"finished"<<endl;
	stalePos = false;
}

void speedCallback(const eecs376_msgs::CrawlerDesiredState::ConstPtr& newSpeed) 
{
	cout<< "steering speed callback happened"<<endl;
	desired = *newSpeed;
        //desired.header = newSpeed.header;
	//desired.des_pose = newSpeed.des_pose;
	//desired.des_speed = newSpeed.des_speed;
	//desired.des_rho = newSpeed.des_rho;
	//desired.des_lseg = newSpeed.des_lseg;
	//desired.seg_type = newSpeed.seg_type;
	//desired.seg_number = newSpeed.seg_number;
	staleDes = false;
}

inline double coerceAngle(double angle){
	if (angle<-pi) {return angle + 2*pi;}
	if (angle> pi) {return angle - 2*pi;}
	return angle;
}

inline double norm(double x1,double y1,double x2,double y2){
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
inline double norm(double x,double y){
	return sqrt(x*x+y*y);
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
	
	double v, w;
	cv::Vec3d sdp;
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
//		cout<<"Steering activate!"<<endl;
		
		sdp = calculateSteeringParameters(&poseActual.pose, &desired.des_pose);			
		switch(desired.seg_type){
			case 1:	//line
			{
				cout<<"STEERING IN A LINE\n";
				cout<<"desired: "<<desired.des_pose<<endl;
				cout<<"actual: "<<poseActual.pose<<endl;
				//double x_err = poseActual.pose.position.x - desired.des_pose.position.x;
				//double y_err = poseActual.pose.position.y - desired.des_pose.position.y;
				//double theta = tf::getYaw(desired.des_pose.orientation) - atan2(y_err,x_err);

				//double d_err = sin(theta) * norm(x_err,y_err);
				//double s_err = cos(theta) * norm(x_err,y_err);
				//double p_err=coerceAngle(tf::getYaw(poseActual.pose.orientation) - tf::getYaw(desired.des_pose.orientation));
				//cout<<" d s p: "<<d_err<<","<<s_err<<","<<p_err<<endl;
				v = desired.des_speed - kS * sdp[0];
				w = -kD*sdp[1]-kP*sdp[2];
				break;
			}

			case 2:	//arc
			{
				cout<<"STEERING IN A ARC"<<endl;
				double r = 1.f / desired.des_rho;
//				double theta = tf::getYaw(desired.des_pose.orientation) + pi/2 * (r>0?1:-1);			//heading to arc center from desired
//				double cX = desired.des_pose.position.x + r * cos(theta);					//x coordinate of arc center
//				double cY = desired.des_pose.position.y + r * sin(theta);					//y coordinate of arc center
//				double pX = cX + r * cos(atan2(poseActual.pose.position.y - cY,poseActual.pose.position.x-cX));	//x coordinate of projection onto arc
//				double pY = cY + r * sin(atan2(poseActual.pose.position.y - cY,poseActual.pose.position.x-cX));	//y coordinate of projection onto arc
//				double p = coerceAngle(atan2(pY-cY,pX-cX) + pi/2 * (r>0?1:-1));					//heading of projection onto arc

//				double p_err = coerceAngle(tf::getYaw(poseActual.pose.orientation) - p); 
//				double s_err = r * coerceAngle(p - tf::getYaw(desired.des_pose.orientation));
//				double d_err = norm(pX,pY,poseActual.pose.position.x,poseActual.pose.position.y) * (norm(poseActual.pose.position.x-cX,poseActual.pose.position.y-cY)>r?1:-1);

				v = desired.des_speed + kS * sdp[0];
				w = v / r + (kD*sdp[1]+kP*sdp[1])*(r>0?1:-1);
				break;
			}

			case 3:	//turn in place
			{
				cout<<"STEERING IN A CIRCLE"<<endl;	
				//double s_err = tf::getYaw(desired.des_pose.orientation) - tf::getYaw(poseActual.pose.orientation);

				v = 0;
				w = desired.des_speed - kS * sdp[0];
				break;
			}

			default:
				cout<<"STEERING IN A FAIL"<<endl;
				v = vel_object.linear.x;
				w = vel_object.angular.z;
		}
			cout<<"Steering calculated"<<endl;
				//limit velocities
			v = max(0,min(v,1));
			w = w<0? max(w,-1):min(w,1);

			vel_object.linear.x = v;
			vel_object.angular.z= w;
			stalePos = true,
		     	staleDes = true;

			vel_object.linear.x = v;
			vel_object.angular.z = w;
			cout<<"steering:\n\tNominalSpeed "<<desired.des_speed<<"\n\tcommanded "<<v<<" , "<<w<<endl;
			pub.publish(vel_object);
	}
}
