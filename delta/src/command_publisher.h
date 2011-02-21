#include <ros/ros.h>
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

typedef struct {
	Pose p;
	double orientation;
	double speed;
	double rotation;
} PoseState;


struct PathList {
	int segNum;
	double segLen;
	int segType;
	Pose referencePt;
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

cv::Mat_<bool>* getMap(OccupancyGrid&);


		
