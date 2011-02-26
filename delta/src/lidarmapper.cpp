#include<ros/ros.h>
#include<tf/transform_datatypes.h> // for tf::getYaw
#include<tf/transform_listener.h> // for the TransformListener class that abstracts away a lot of tf
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h> //for the laser projector class
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <math.h>

#include "CSpaceFuncs.h"

#define REFRESH_RATE 10 //hz

#define GRID_RES .05 // 10 cm grid
#define GRID_WIDTH 100// meters wide
#define GRID_HEIGHT 100 // meters tall... should be plenty of space
//#define GRID_PADDING 1 //meters of padding to try to avoid buffer overflows.
#define NUM_WIDTH ceil(GRID_WIDTH/GRID_RES)  //number of squares wide and tall for occupancy grid.
#define NUM_HEIGHT ceil(GRID_HEIGHT/GRID_RES)

#define DONUT_RADIUS 2.54*.210/2.0 //the radius of the donut in m
#define DONUT_LENGTH ((DONUT_RADIUS)*2.0) //the size length of the side of a square to make to contain the donut rounded up to the nearest m.
#define NUM_DONUT ((int)2*ceil(DONUT_LENGTH/(2.0*GRID_RES)))

sensor_msgs::PointCloud last_map_cloud;
nav_msgs::OccupancyGrid Output;
int** JellyDonut;
tf::TransformListener *tfl;
double PoseX;
double PoseY;
double InitX;
double InitY;
geometry_msgs::PoseStamped last_map_pose;
nav_msgs::Odometry last_odom;

using namespace std;

int Address(int i, int j)
{
	return i*NUM_WIDTH+j;
}

int Address(double X, double Y)
{
	X -=Output.info.origin.position.x;
	Y -=Output.info.origin.position.y;
	int x = X/GRID_RES;
	int y = Y/GRID_RES;
	return Address(y,x);
}

void CopyPoints()
{

	int numPts = last_map_cloud.points.size();
	for(int i = 0; i<numPts; i++)
	{
		double X = last_map_cloud.points[i].x;
		double Y = last_map_cloud.points[i].y;
		if (Y>InitY-GRID_WIDTH/2.0+DONUT_LENGTH/2.0&&Y<InitY+GRID_WIDTH/2.0-DONUT_LENGTH/2.0
		&&X>InitX-GRID_HEIGHT/2.0+DONUT_LENGTH/2.0&&X<InitX+GRID_HEIGHT/2.0-DONUT_LENGTH/2.0)
		{
			for (int j =1; j<DONUT_LENGTH; j++)
			{
				double x = X-DONUT_LENGTH/2.0+j*GRID_RES;
				double y = Y-DONUT_LENGTH/2.0+i*GRID_RES;
				for (int k = 1; k<DONUT_LENGTH; k++)
				{
					Output.data[Address(x,y)]=Output.data[Address(x,y)]||JellyDonut[j][k];
				}
			}
		}
		
	}
}

// [i,j] = i*width+j

void GridInit()
{
	Output.header.seq = 0;
	Output.header.frame_id = "map";
	//Output.header.stamp = time(NULL);
	Output.info.resolution = .05;
	//Output.info.map_load_time = time(NULL);
	Output.info.width = NUM_WIDTH;
	Output.info.height = NUM_HEIGHT;
	Output.info.origin.position.x = InitX-GRID_WIDTH/2.0;
	Output.info.origin.position.y = InitY-GRID_HEIGHT/2.0;
	Output.info.origin.position.z = 0;
	Output.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
	vector<char>* data = new vector<char>((NUM_WIDTH) * (NUM_HEIGHT));
	Output.data.assign(data->begin(),data->end()); //I realize that this size should be 8 by definition, but this is good practice.
}

void DonutInit()
{
	JellyDonut = (int **)calloc(NUM_DONUT, sizeof(int));
	for (int i = 0; i< NUM_DONUT; i++)
	{
		JellyDonut[i] = (int*) calloc(NUM_DONUT, sizeof(int));
	}
	
	double x =0;
	double y =0;
	for (int i =0;  i <NUM_DONUT; i++)
	{
		for(int j = 0; j<NUM_DONUT; j++)
		{
			x = (i - DONUT_LENGTH/(2.0))/GRID_RES;
			y = (j - DONUT_LENGTH/(2.0))/GRID_RES;
			if (x*x+y*y<DONUT_LENGTH)
			{
				JellyDonut[i][j] = 100;
			}
		}
	}
	
}

bool init = false;
geometry_msgs::PoseStamped temp;
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) 
{
		last_odom = *odom;
        temp.pose = last_odom.pose.pose;
        temp.header = last_odom.header;
	cout<<"temp "<<temp.pose.position.x<<" , "<<temp.pose.position.y<<endl;
        try 
	{
          tfl->transformPose("map", temp, last_map_pose);
        } 
	catch (tf::TransformException ex) 
	{
	  cout << "We caught an error!" << endl;
          ROS_ERROR("%s", ex.what());
        }
	
	PoseX = last_map_pose.pose.position.x;
	PoseY = last_map_pose.pose.position.y;
	
	if(init == false)
	{
		InitX = PoseX;
		InitY = PoseY;
		GridInit();
	}
	
	init = true;
}

void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& scan_cloud) 
{
	if (init == false)
	{
		//cout<<"2callback\n";
		last_map_cloud = *scan_cloud; 
		//ROS_INFO("I got a scan cloud of size %lu", last_map_cloud.points.size());
		CopyPoints();
		//cout<<"yo\n";
	}
}

int main(int argc,char **argv)
{
	cout<<"2\n";
	DonutInit();
	cout<<"2\n";
	ros::init(argc,argv,"lidar_mappa");//name of this node
	tfl = new tf::TransformListener();
	cout<<"2\n";
	ros::Rate naptime(REFRESH_RATE); //will perform sleeps to enforce loop rate of "10" Hz
	while (!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce();
	cout<<"2\n";
	ros::NodeHandle n;
	ros::Subscriber S1 = n.subscribe<sensor_msgs::PointCloud>("LIDAR_Cloud", 20, cloudCallback);
	ros::Subscriber S2 = n.subscribe<nav_msgs::Odometry>("Pose_Actual", 10, odomCallback);
	ros::Publisher P = n.advertise<nav_msgs::OccupancyGrid>("LIDAR_Map", 10);
	cout<<"2\n";
	while(true)
	{
		//cout<<"2\n";
		ros::spinOnce(); //spin until ctrl-C or otherwise shutdown
		P.publish(Output);
		//cout<<"2\n";
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (100ms) period
		//thus enforcing that we achieve the desired update rate (10Hz)
	}
	
	return 0; // this code will only get here if this node was told to shut down, which is
	// reflected in ros::ok() is false 
}
