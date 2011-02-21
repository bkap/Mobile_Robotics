#include <ros/ros.h>
<<<<<<< HEAD
#include "opencv2/core/core.hpp"
#include <nav_msgs/OccupancyGrid.h>
#define CSPACE_RESOLUTION 0.05
using namespace nav_msgs;
struct Pose {
	double x;
	double y;
	double psi;
	
	Pose(double x, double y, double psi) {
		this->x = x;
		this->y = y;
		this->psi = psi;
	}
};
=======
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cv.h>
#define CSPACE_RESOLUTION = 0.05;

using namespace cv;
>>>>>>> b48908e7b17ef3ecab1371c518cf351b70a04d91

typedef struct {
	geometry_msgs::Pose p;
	double orientation;
	double speed;
	double rotation;
} PoseState;


struct PathList {
	int segNum;
	double segLen;
	int segType;
    geometry_msgs::Pose referencePt;
	double initTanAng;
	double curvature;
	double max_speed;
	double max_spin;
	double max_accel;
	double max_spin_accel;
};

typedef struct {
	double lsegDes;
	double xDes;
	double yDes;
	double psiDes;
	double rhoDes;
	int segType;
	int segNumber;
	double speedNominal;
} CrawlerDesState;

<<<<<<< HEAD
cv::Mat_<bool>* getMap(OccupancyGrid&);
=======
cv::Mat& getMap(nav_msgs::OccupancyGrid grid);
>>>>>>> b48908e7b17ef3ecab1371c518cf351b70a04d91


		
