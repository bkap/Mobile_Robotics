#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include<tf/transform_listener.h>
#include<nav_msgs/Odometry.h>



using namespace nav_msgs;
using namespace geometry_msgs;
ros::Publisher pose_pub;
ros::Subscriber odom_sub;
tf::TransformListener *tfl;
PoseStamped temp;
void odomCallback(const Odometry::ConstPtr& odom) {
	PoseStamped output;
	Odometry o = *odom;
	temp.pose = o.pose.pose;
	temp.header = o.header;
	try {
			tfl->transformPose("map",temp,output);
			pose_pub.publish(output);
	} catch(tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}

}

int main(int argc, char** argv) {
	ros::init(argc,argv, "pso2");
	ros::NodeHandle n;

	tfl = new tf::TransformListener();
	
	while(!ros::Time::isValid()) { ros::spinOnce(); }
	while(!tfl->canTransform("map","odom", ros::Time::now())) {
		ros::spinOnce();
	}
	odom_sub = n.subscribe<Odometry>("odom",1,odomCallback);
	pose_pub = n.advertise<PoseStamped>("poseActual",1);
	
	ROS_INFO("fakepso starting");
	ros::spin();
}
