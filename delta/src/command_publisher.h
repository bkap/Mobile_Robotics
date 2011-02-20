#ifndef _COMMAND_PUB
#define _COMMAND_PUB
#include <ros/ros.h>
#define CSPACE_RESOLUTION = 0.05;
struct Pose {
	double x;
	double y;
	double psi;
	
	Pose(double x, double y, double psi) {
		this.x = x;
		this.y = y;
		this.psi = psi;
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

cv::Mat& getMap(OccupancyGrid);


#endif
		
