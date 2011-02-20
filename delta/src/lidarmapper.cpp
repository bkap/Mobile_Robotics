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
#define GRID_WIDTH 30// 30 meters wide
#define GRID_HEIGHT 30 //30 meters tall... should be plenty of space
#define NUM_WIDTH ceil(GRID_WIDTH/GRID_RES)  //number of squares wide and tall for occupancy grid.
#define NUM_HEIGHT ceil(GRID_HEIGHT/GRID_RES)

sensor_msgs::PointCloud last_map_cloud;
nav_msgs::OccupancyGrid Output;
tf::TransformListener *tfl;

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
	Output.origin.position.x = 0;
	Output.origin.position.y = 0;
	Output.origin.position.z = 0;
	Output.origin.orientation = tf::createQuaternionMsgFromYaw(0);
	data = (int*) calloc(NUM_WIDTH*NUM_HEIGHT, sizeof(int8)); //I realize that this size should be 8 by definition, but this is good practice.
	for(int i = 0; i<NUM_WIDTH; i++)
	{
		
	}
	for (int i = 0; i<NUM_HEIGHT; i++)
	{
		
	}
}

void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& scan_cloud) {
  last_map_cloud = *scan_cloud; 
  ROS_INFO("I got a scan cloud of size ", last_map_cloud.points.size());
  
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"lidar_listener");//name of this node
  GridInit();
  tfl = new tf::TransformListener();
  while (!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce();
  ros::NodeHandle n;
  ros::Subscriber S = n.subscribe("LIDAR_Cloud", 1, cloudCallback);
  ros::Publisher P = n.advertise("LIDAR_Cloud", 1);
  ros::spin(); //spin until ctrl-C or otherwise shutdown
  return 0; // this code will only get here if this node was told to shut down, which is
  // reflected in ros::ok() is false 
}