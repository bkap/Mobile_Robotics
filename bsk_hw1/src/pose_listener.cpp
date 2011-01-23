#include <ros/ros.h>
#include <cwru_base/Pose.h>
#include <iostream>
using namespace std;
void poseCallback(const cwru_base::Pose::ConstPtr& pose) {
	cout << pose->x<<endl;
}
int main(int argc, char **argv) {
	ros::init(argc, argv, "pose_listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("flipped_pose",1,poseCallback);
	ros::spin();
	return 0;
}
