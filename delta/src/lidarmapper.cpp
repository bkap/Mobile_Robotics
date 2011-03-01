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
#include <iostream>		
#include "CSpaceFuncs.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#define pi 3.14159265358979323846264338327950288



/*TODO
	tie grid position to pose
*/

const double loopRate = 10;

const double gridOx = -10;	//origin y co-ordinate
const double gridOy = -10;	//origin x co-ordinate
const double gridRes = 0.05;	//5cm per pixel
const double patchRadius = 15/39.37;//radius of fattening patch (robot radius)
const double gridLength = 45;	//25 meter square
const int gridSize = ceil(gridLength / gridRes);	//pixel width of grid
const int patchSize = 2*ceil(patchRadius/gridRes) + 1;	//pixel width of patch

sensor_msgs::PointCloud scanCloud;
nav_msgs::OccupancyGrid cSpace;
int** patch;			//fattening template

/*presently unused, will implement later*/
	double PoseX;			
	double PoseY;
	double InitX;
	double InitY;
	geometry_msgs::PoseStamped last_map_pose;
	nav_msgs::Odometry last_odom;

using namespace std;

inline bool inGrid(double x, double y){
	return x>=gridOx && x<= gridOx+gridLength
		&&y>=gridOy && y<=gridOy+gridLength;
}

inline bool fatInGrid(double x, double y){
	return inGrid(x+patchRadius+gridRes,y)&&inGrid(x-patchRadius-gridRes,y)
		&&inGrid(x,y+patchRadius+gridRes)&&inGrid(x,y-patchRadius-gridRes);
}

inline int address(int x, int y)
{
	//cout<< "\tplacing ("<<x<<","<<y<<") in ["<<y * gridSize + x<<"]"<<endl;
	return y * gridSize + x;
}
void cSpaceInit()
{
	cout<<"creating cSpace grid:"<<endl;
	cSpace.header.seq = 0;
	cSpace.header.frame_id = "map";
	//Output.header.stamp = time(NULL);
	cSpace.info.resolution = gridRes;
	//Output.info.map_load_time = time(NULL);
	cSpace.info.width = gridSize;//NUM_WIDTH;
	cSpace.info.height = gridSize;//NUM_HEIGHT;
	cSpace.info.origin.position.x = gridOx;//InitX-GRID_WIDTH/2.0;
	cSpace.info.origin.position.y = gridOy;//InitY-GRID_HEIGHT/2.0;
	cSpace.info.origin.position.z = 0;
	cSpace.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
	vector<char>* data = new vector<char>((cSpace.info.width) * (cSpace.info.height));
	cSpace.data.assign(data->begin(),data->end()); //I realize that this size should be 8 by definition, but this is good practice.
	cout<<"\tcreated cSpace grid with "<<cSpace.info.width*cSpace.info.width<<" elements"<<endl;
}

void patchInit()
{
	patch = (int **)calloc(patchSize, sizeof(int));
	//cout<<"creating patch of size "<<patchSize<<endl;
	int center = (patchSize-1)/2;	//center pixel coordinate
	int rsquared = center*center;
	for (int i = 0; i< patchSize; i++)
	{
		patch[i] = (int*) calloc(patchSize, sizeof(int));
		for(int j=0;j<patchSize;j++)
		{
			if((i-center)*(i-center) + (j-center)*(j-center) < rsquared)
			{
				patch[i][j] = 100;
			}
		}
	}
}

void copyPoints()	
{
	static cv::Mat image = cv::Mat(gridSize,gridSize,CV_8U);
	//cout<<"copying points:"<<endl;
	int numPts = scanCloud.points.size();
	cv::Vec2d r;
	double theta;
	for(int i = 0;i<numPts;i++)
	{
		double x = scanCloud.points[i].x;
		double y = scanCloud.points[i].y;

		r = cv::Vec2d(x - last_map_pose.pose.position.x,y-last_map_pose.pose.position.y);
		theta=  tf::getYaw(last_map_pose.pose.orientation) - atan2(r[1],r[0]);
		if(theta < - pi/2 || theta > pi/2  || r.dot(r) > 100)	{continue;}	//only accept points from within 10m in the correct directions in case a transform broke
		if(fatInGrid(x,y))
		{
			
			//cout<<"\tpoint validated at (" << x<<","<<y<<")"<<endl;
			int Gx = round((x - gridOx)/gridRes) - (patchSize-1)/2;
			int Gy = round((y - gridOy)/gridRes) - (patchSize-1)/2;
			for(int j = 0; j < patchSize;j++)	//rows - y
			{
				for(int k = 0; k < patchSize;k++)//cols - x
				{
					cSpace.data[address(Gx + k ,Gy + j)] = cSpace.data[address(Gx + k, Gy + j)] | patch[j][k];
					image.at<char>(Gy+j,Gx+k)=cSpace.data[address(Gx + k ,Gy + j)];
				}
			}
		}
	}
	cv::imshow("cSpace",image);
	waitKey(5);
	//cout<<"copied points"<<endl;
}

bool init = false;
geometry_msgs::PoseStamped temp;
tf::TransformListener *tfl;
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) 
{
//	cout<<"lidarmapper odom callback occured\n";
	last_odom = *odom;
        temp.pose = last_odom.pose.pose;
        temp.header = last_odom.header;
	//cout<<"temp "<<temp.pose.position.x<<" , "<<temp.pose.position.y<<endl;
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
		cout<<"odom callback initialized"<<endl;
		InitX = PoseX;
		InitY = PoseY;
		cSpaceInit();
		init = true;
	}
	
}

void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& scan_cloud) 
{
	if (init == true)
	{
		//cout<<"2callback\n";
		scanCloud = *scan_cloud; 
		//ROS_INFO("I got a scan cloud of size %lu", scanCloud.points.size());
		copyPoints();
		//cout<<"yo\n";
	}
}

int main(int argc,char **argv)
{
	cout<<"2\n";
	patchInit();
//	cSpaceInit();
//	init=true;
	cout<<"2\n";
	ros::init(argc,argv,"lidar_mapper");//name of this node
	tfl = new tf::TransformListener();
	cout<<"2\n";
	ros::Rate loopTimer(loopRate); //will perform sleeps to enforce loop rate of "10" Hz
	while (!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce();
	cout<<"2\n";
	ros::NodeHandle n;
	ros::Subscriber S1 = n.subscribe<sensor_msgs::PointCloud>("LIDAR_Cloud", 20, cloudCallback);
	ros::Subscriber S2 = n.subscribe<nav_msgs::Odometry>("odom", 10, odomCallback);
	ros::Publisher P = n.advertise<nav_msgs::OccupancyGrid>("LIDAR_Map", 10);
	cout<<"2\n";
	namedWindow("cSpace",CV_WINDOW_NORMAL);
	while(ros::ok())
	{
		//cout<<"2\n";
		ros::spinOnce(); //spin until ctrl-C or otherwise shutdown
		//cout<<"grid dimensions "<<cSpace.info.width<<" by "<< cSpace.info.height<<endl;
		P.publish(cSpace);
		//cout<<"2\n";
		loopTimer.sleep(); // this will cause the loop to sleep for balance of time of desired (100ms) period
		//thus enforcing that we achieve the desired update rate (10Hz)
	}
	
	return 0; // this code will only get here if this node was told to shut down, which is
	// reflected in ros::ok() is false 
}

