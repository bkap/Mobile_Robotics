#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_listener.h>
#include <fstream>
#include <iostream>
#include "cvFuncs.h"

using namespace cv;
using namespace geometry_msgs;
using namespace std;

/*
TODO
	can be made faster and more accurate,
		notably by directly extracting yaw in certain cases
		and by less use of internal functions

*/
template <typename T>
void readMat(cv::Mat_<T>& mat, char* file){
	ifstream* infile = new ifstream(file,ifstream::in&ifstream::binary);
	int rows = 0,cols = 0;
	(*infile)>>rows;
	(*infile)>>cols;
	mat = Mat_<T>(rows,cols);
	T val;
	for(int i=0;i<mat.rows;i++){
		for(int j=0;j<mat.cols;j++){
			(*infile)>>val;
			mat(i,j)=val;
		}
	}
	infile->close();
}
template <typename T>
void writeMat(cv::Mat_<T>& mat, char* file){
	ofstream* ofile = new ofstream(file,ofstream::out&ofstream::binary);
	(*ofile)<<mat.rows<<" ";
	(*ofile)<<mat.cols<<" ";
	
	for(int i = 0;i<mat.rows;i++){
		for(int j = 0;j<mat.cols;j++){
			(*ofile)<<(T) mat(i,j)<<" ";
		}
	}
	ofile->close();
}



void setVec(Vec2f& vec, float x, float y){
	vec[0] = x;
	vec[1] = y;
}

void setVec(Vec3f& vec, float x, float y, float z){
	vec[0] = x;
	vec[1] = y;
	vec[2] = z;
}

void setVec(Vec2d& vec, double x, double y){
	vec[0] = x;
	vec[1] = y;
}

void setVec(Vec3d& vec, double x, double y, double z){
	vec[0] = x;
	vec[1] = y;
	vec[2] = z;
}

float angle(Vec2f vec){
	return CV_PI / 180.0 * fastAtan2(vec[1],vec[0]);
}

double angle(Vec2d vec){
	return CV_PI / 180.0 * fastAtan2(vec[1],vec[0]);
}

void getUnitVec(Vec2d& dir, double psi){
	setVec(dir,cos(psi),sin(psi));
}	

void getUnitVec(Vec2f& dir, double psi){
	setVec(dir,cos(psi),sin(psi));
}

void ROS2CVPointCloud(const sensor_msgs::PointCloud& cloud, vector<Point3f>& points){
	for(int i=0;i<cloud.points.size();i++){
		points.push_back(Point3f(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z));
	}
}

void ROS2CVPose(Pose& pose, Vec2f& pos, Vec2f& dir){
	double theta = tf::getYaw(pose.orientation);
	setVec(pos, pose.position.x, pose.position.y);
	setVec(dir, cos(theta), sin(theta));
}

void CV2ROSPose(Vec2f& pos, Vec2f& dir, Pose& pose){
	pose.orientation = tf::createQuaternionMsgFromYaw(atan2(dir[1],dir[0]));
	pose.position.x = pos[0];
	pose.position.y = pos[1];
}

void traverseDistAlongArc(Pose& init, Pose& ref, double dist, Pose& dest){
	Vec2f initDir, initPos, refDir, refPos;
	ROS2CVPose(init,initPos,initDir);
	ROS2CVPose(ref,refPos,refDir);
	double rad = dist * sin(angle(initDir) - angle(refDir))/ (2 * CV_PI * norm(initPos-refPos));
	traverseRadAlongArc(init,ref,rad, dest);
}

void traverseRadAlongArc(Pose& init, Pose& ref, double rad, Pose& dest){
	Vec2f initDir,initPos,refDir,refPos,destDir,destPos;
	ROS2CVPose(init,initPos,initDir);
	ROS2CVPose(ref,refPos,refDir);

	Vec2f r,rDir,newDir;
	r = initPos - refPos;
	getUnitVec(rDir,angle(r));
	getUnitVec(newDir,angle(r)+rad);
	destPos = newDir * (float) norm(r);
	destPos = destPos + refPos;
	getUnitVec(destDir,angle(initDir)+rad);
	CV2ROSPose(destPos,destDir,dest);
}

void traverseDistAlongLine(Pose& init, double dist,Pose& dest){
	Vec2f initDir, initPos,destDir,destPos;
	ROS2CVPose(init,initPos,initDir);
	destDir = initDir;
	destPos = initPos + (float)dist * initDir;
	CV2ROSPose(destPos,destDir,dest);
}

void traverseRadAlongCorner(Pose& init, double rad,Pose& dest){
	Vec2f initDir,initPos,destDir,destPos;
	ROS2CVPose(init,initPos,initDir);
	destPos = initPos;
	getUnitVec(destDir,angle(initDir)+rad);	
	CV2ROSPose(destPos, destDir,dest);
}
