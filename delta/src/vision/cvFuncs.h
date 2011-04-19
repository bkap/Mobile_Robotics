#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_listener.h>
#ifndef incl_cvFuncs
#define incl_cvFuncs 1
void setVec(cv::Vec2f& vec, float x, float y);
void setVec(cv::Vec3f& vec, float x, float y, float z);
void setVec(cv::Vec2d& vec, double x, double y);
void setVec(cv::Vec3d& vec, double x, double y, double z);

float angle(cv::Vec2f dir);

double angle(cv::Vec2d dir);

void getUnitVec(cv::Vec2d& dir, double psi);

void getUnitVec(cv::Vec2f& dir, double psi);

void ROS2CVPose(geometry_msgs::Pose& pose, cv::Vec2f& pos, cv::Vec2f& dir);

void CV2ROSPose(cv::Vec2f& pos, cv::Vec2f& dir, geometry_msgs::Pose& pose);

void traverseDistAlongArc(geometry_msgs::Pose& init, geometry_msgs::Pose& ref, double dist, geometry_msgs::Pose& dest);

void traverseRadAlongArc(geometry_msgs::Pose& init, geometry_msgs::Pose& ref, double rad, geometry_msgs::Pose& dest);

void traverseDistAlongLine(geometry_msgs::Pose& init, double dist,geometry_msgs::Pose& dest);

void traverseRadAlongCorner(geometry_msgs::Pose& init, double rad,geometry_msgs::Pose& dest);

template <typename T>
void writeMat(cv::Mat_<T>& mat, char* file);

template <typename T>
void readMat(cv::Mat_<T>& mat, char* file);
#endif
