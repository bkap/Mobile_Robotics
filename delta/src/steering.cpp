#include<ros/ros.h>
#include <geometry_msg/Twist.h>
#include <math.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_listener.h>

#define pi 3.14159265358979323846264338327950288
	
const double LOOP_RATE = 25;

bool stalePos = true,
     staleDes = true;


geometry_msgs::PoseStamped poseActual;
cwru_wsn_steering::DesiredState desired;
tf::TransformListener *tfl;

void PSOCallback(const geometry_msgs::PoseStamped& odom) 
{
        cout<<"temp "<<odom.pose.position.x<<" , "<<odom.pose.position.y<<endl;
        try 
	{
          tfl->transformPose("map", odom, poseActual);
        } 
	catch (tf::TransformException ex) 
	{
	  cout << "We caught an error!" << endl;
          ROS_ERROR("%s", ex.what());
        }
	stalePos = false;
}
void speedCallback(const cwru_wsn_steering::DesiredState& newSpeed) 
{
        desired.header = newSpeed.header;
	desired.x = newSpeed.x;
	desired.y = newSpeed.y;
	desired.theta = newSpeed.theta;
	desired.rho = newSpeed.rho;
	desired.v = newSpeed.v;
	staleDes = false;
}

inline double coerceAngle(double angle){
	if (angle<-2*pi) {return angle + 2*pi;}
	if (angle> 2*pi) {return angle - 2*pi;}
}

inline double norm(double x1, y1,x2, y2){
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
inline double norm(double x, y){
	return sqrt(x*x+y*y);
}

int main(int argc,char **argv)
{
	
	ros::init(argc,argv,"steering");//name of this node
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	ros::Subscriber sub1 = n.subscribe<geometry_msgs::PoseStamped>("PSO", 1, PSOCallback);
	ros::Subscriber sub1 = n.subscribe<geometry_msgs::PoseStamped>("Crawler", 1, crawlerCallback);
	ros::Subscriber sub1 = n.subscribe<geometry_msgs::PoseStamped>("NominalSpeed", 1, speedCallback);

	geometry_msgs::Twist vel_object;
	ros::Duration elapsed_time; // define a variable to hold elapsed time
	ros::Rate loopTimer(LOOP_RATE); //will perform sleeps to enforce loop rate of "10" Hz
	while (!ros::Time::isValid()) ros::spinOnce(); // simulation time sometimes initializes slowly. Wait until ros::Time::now()

	double v, w;

	const int line = 1;
	const int arc  = 2;
	const int turn = 3;
	while(ros::OK()){
		naptime.sleep();
		ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely

		ros::Time current_time = ros::Time::now();
	 	desired.header.stamp = current_time;
		elapsed_time= ros::Time::now()-birthday;
		ROS_INFO("birthday is %f", birthday.toSec());
		ROS_INFO("elapsed time is %f", elapsed_time.toSec());	

		if(stalePos)	{continue;}

		switch(crumb.segType){
			case line:
				double x_err = desired.x - poseActual.pose.position.x;
				double y_err = desired.y - poseActual.pose.position.y;
				double theta = desired.theta - atan2(y_err,x_err);

				double d_err = sin(theta) * norm(x_err,y_err);
				double s_err = cos(theta) * norm(x_err,y_err);
				double p_err=coerceAngle(tf::getYaw(poseActual.pose.orientation) - desired.theta);
	
				v = speedNominal + kV * s_err;
				w = kD*d_err+kP*p_err;
				break;

			case arc:
				double r = 1.f / desired.rho;
				double theta = desired.theta + pi/2 * (r>0?1:-1);		//heading to arc center from desired
				double cX = desired.x + r * cos(theta);				//x coordinate of arc center
				double cY = desired.y + r * sin(theta);				//y coordinate of arc center
				double pX = cX + r * cos(atan2(poseActual.pose.position.y - cY,poseActual.pose.position.x-cX));	//x coordinate of projection onto arc
				double pY = cY + r * sin(atan2(poseActual.pose.position.y - cY,pos.pose.position.x-cX));	//y coordinate of projection onto arc
				double p = coerceAngle(atan2(pY-cY,pX-cX) + pi/2 * (r>0?1:-1));				//heading of projection onto arc

				double p_err = coerceAngle(tf::getYaw(pos.pose.orientation) - p); 
				double s_err = r * coerceAngle((p - tf::getYaw((*desired).pose.orientation));
				double d_err = norm(pX,pY,pos.pose.position.x,pos.pose.position.y) * (norm(pos.pose.position.x-cX,pos.pose.position.y-cY)>r?1:-1);
		
		
				v = speedNominal + kV * s_err;
				w = v / r + kD*d_err+kP*p_err;
				break;

			case turn:
				double s_err = tf::getYaw((*desired).pose.orientation) - tf::getYaw((pos.pose.orientation);

				v = 0;
				w = speedNominal + kV * s_err;
				break;

			default:
				v = vel_object.linear.x;
				w = vel_object.angular.z;
		}
				//limit velocities
			v = max(0,min(v,desired.maxV));
			w = (w<0? max(w,desired.maxW):min(w,desired.maxW);

			vel_object.linear.x = v;
			vel_object.angular.z= w;
			stalePos = true,
		     	staleDes = true;
			
			vel_object.linear.x = v;
			vel_object.angular.z = w;
					
			pub.publish(vel_object);
	}
}
