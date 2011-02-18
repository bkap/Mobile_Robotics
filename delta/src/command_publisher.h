#ifndef _COMMAND_PUB
#define _COMMAND_PUB
#include <ros/ros.h>
#include <geometry_msgs/PoseStampted.h>

typedef struct {
	double x;
	double y;
	double psi;
} Pose;

typedef struct {
	Pose p;
	double orientation;
	double speed;
	double rotation;
} PoseState;

typedef struct PathList path_list_t;

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
	path_list_t* next_seg;
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




#endif
		
