#include<ros/ros.h>
#include<tf/transform_datatypes.h> // for tf::getYaw
#include<tf/transform_listener.h> // for the TransformListener class that abstracts away a lot of tf
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h> //for the laser projector class
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <math>
#include "cv.h"

#define GRID_RES .05 // 5 cm grid
#define GRID_WIDTH 10// meters wide
#define GRID_HEIGHT 10 // meters tall... should be plenty of space
#define GRID_PADDING 1 //meters of padding to try to avoid buffer overflows.
#define NUM_WIDTH ceil((GRID_WIDTH+2.0*GRID_PADDING)/GRID_RES)  //number of squares wide and tall for occupancy grid.
#define NUM_HEIGHT ceil((GRID_HEIGHT+2.0*GRID_PADDING)/GRID_RES)

#define DONUT_RADIUS 2.54*.210/2.0 //the radius of the donut in m
#define DONUT_LENGTH (int)(ceil(DONUT_RADIUS)*2.0) //the size length of the side of a square to make to contain the donut rounded up to the nearest m.

sensor_msgs::PointCloud last_map_cloud;
nav_msgs::OccupancyGrid Output;
int** JellyDonut;
tf::TransformListener *tfl;
double PoseX;
double PoseY;

void CopyPoints()
{

	int numPts = last_map_cloud.points.size();
	for(int i = 0; i<numPts; i++)
	{
		if (last_map_cloud.points[i].y>PoseY-GRID_WIDTH/2.0-GRID_PADDING&&last_map_cloud.points[i].y<PoseY+GRID_WIDTH/2.0+GRID_PADDING)
		&&last_map_cloud.points[i].x>PoseX-GRID_HEIGHT/2.0-GRID_PADDING&&last_map_cloud.points[i].x<PoseX+GRID_HEIGHT/2.0+GRID_PADDING)
		{
			for (int j =0; j<DONUT_LENGTH; j++)
			{
				for (int k = 0; k<DONUT_LENGTH; k++)
				{
					Output[Address(k-DONUT_LENGTH/(2.0*GRID_RES),j-DONUT_LENGTH/(2.0*GRID_RES));
				}
			}
		}
		
	}
}

// [i,j] = i*width+j

int Address(int i, int j)
{
	return i*NUM_WIDTH+j;
}

void GridInit()
{
	Output.header.seq = 0;
	Output.header.frame = "map"
	Output.header.stamp = time(NULL);
	Output.MapMetaData.resolution = .05;
	Output.MapMetaData.map_load_time = time(NULL);
	Output.MapMetaData.Width = NUM_WIDTH;
	Output.MapMetaData.Height = NUM_HEIGHT;
	Output.origin.position.x = PoseX-GRID_WIDTH/2.0-GRID_PADDING;
	Output.origin.position.y = PoseY-GRID_HEIGHT/2.0-GRID_PADDING;
	Output.origin.position.z = 0;
	Output.origin.orientation = tf::createQuaternionMsgFromYaw(0);
	Output.data = (int*) calloc((NUM_WIDTH)*(NUM_HEIGHT), sizeof(int8)); //I realize that this size should be 8 by definition, but this is good practice.
}

void DonutInit()
{
	JellyDonut = (int **)calloc(DONUT_LENGTH, sizeof(int));
	for (int i = 0; i< DONUT_LENGTH; i++)
	{
		JellyDonut[i] = (int*) calloc(DONUT_LENGTH, sizeof(int));
	}
	
	double x =0;
	double y =0;
	for (int i =0;  i <DONUT_LENGTH/DONUT_RES; i++)
	{
		for(int j = 0; j<DONUT_LENGTH/DONUT_RES; j++)
		{
			x = (i - DONUT_LENGTH/(2.0*DONUT_RES));
			y = (j - DONUT_LENGTH/(2.0*DONUT_RES));
			if (x*x+y*y<DONUT_RADIUS*DONUT_RADIUS/(DONUT_RES*DONUT_RES))
			{
				JellyDonut[i][j] = 100;
			}
		}
	}
	
}

bool init = false;
void poseActualCallback(const geometry_msgs::PoseStamped::ConstPtr& StartPose)
{
	init = true;
	PoseX = StartPose->X;
	PoseY = StartPose->Y;
}

void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& scan_cloud) 
{
	if (init == false)
	{
		last_map_cloud = *scan_cloud; 
		ROS_INFO("I got a scan cloud of size ", last_map_cloud.points.size());
		GridInit();
		CopyPoints();
		free(Output.data);
	}
}

int main(int argc,char **argv)
{
  DonutInit();
  ros::init(argc,argv,"lidar_listener");//name of this node
  tfl = new tf::TransformListener();
  while (!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce();
  ros::NodeHandle n;
  ros::Subscriber S1 = n.subscribe<sensor_msgs::PointCloud>("LIDAR_Cloud", 1, cloudCallback);
  ros::Subscriber S2 = n.subscribe<geometry_msgs::PoseStamped>("Pose_Actual", 1, poseActualCallback);
  ros::Publisher P = n.advertise("LIDAR_Cloud", 1);
  ros::spin(); //spin until ctrl-C or otherwise shutdown
  return 0; // this code will only get here if this node was told to shut down, which is
  // reflected in ros::ok() is false 
