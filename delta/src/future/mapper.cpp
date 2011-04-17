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

#define CLEAR_RATE 5
#define FILL_RATE 50
#define DECAY_RATE .05

/*TODO

finish and verify the rewrite

manually define sparsity for the sparseMats to save memory

maybe split camera and lidar cspace maps:
	would allow proper raytracing
	would also allow finagling camera cspace population based on presence of lidar cspace obstacle:
		try to fiddle with projection to account for z
		
		would probably be better implemented by polling lidar cspace in camera node before publishing pointcloud


initialize gridOx,gridOy dynamically:
	center start up location in cSpace grid
*/

double loopRate = 10;

const double gridOx = -10;	//origin y co-ordinate
const double gridOy = -10;	//origin x co-ordinate
const double gridRes = 0.05;	//5cm per pixel
const double patchRadius = 15/39.37;//radius of fattening patch (robot radius)
const double gridLength = 45;	//25 meter square

const Rect_<float> gridBounds(gridOx,gridOy,gridLength,gridLength);

const int gridSize = ceil(gridLength / gridRes);	//pixel width of grid
const int patchSize = 2*ceil(patchRadius/gridRes) + 1;	//pixel width of patch

sensor_msgs::PointCloud scanCloud;
nav_msgs::OccupancyGrid cSpace;
cv::Mat_<char> cSpaceMat;
cv::Mat_<char> fattener;
geometry_msgs::PoseStamped last_map_pose;
nav_msgs::Odometry last_odom;

using namespace std;

//initializes the CSpace grid.
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
	cSpaceMat = cv::Mat(cSpace.data,false);
	cSpaceMat = cSpaceMat.reshape(gridSize);
	cout<<"\tcreated cSpace grid with "<<cSpace.info.width*cSpace.info.width<<" elements"<<endl;
}
//one-time generation of patch for fattening
void patchInit(){
	fattener = zeros(patchSize,patchSize,CV_8SC1);
	int center = (patchSize-1)/2;	//center pixel coordinate
	int rsquared = center*center;
	for (int i = 0; i< patchSize; i++){
		for(int j=0;j<patchSize;j++){
			if((i-center)*(i-center) + (j-center)*(j-center) < rsquared){
				fattener(i,j) = 1;
			}
		}
	}
}

int pointToGridIndex(Point2f& point){
	int Gx = round((x - gridOx)/gridRes);
	int Gy = round((y - gridOy)/gridRes);
	return Gy*gridSize + Gx;
}
Vec2i pointToGridCoord(Point2f& point){
	int Gx = round((x - gridOx)/gridRes);
	int Gy = round((y - gridOy)/gridRes);
	return Vec2i(Gy,Gx);
}

vector<Point2f> pointCloudToPoints(const sensor_msgs::PointCloud::ConstPtr& cloud){
	
}

/*
	1 in ROI's, 0 (unpopulated sparseMat values) elsewhere
*/
SparseMat_<char> getLIDARClearROI(){
//should return forward facing 3m radius semicircle

	SparseMat_<char> ROI = SparseMat(gridSize,gridSize,CV_8SC1);
	//may need to offset angles by some reference angle
	double angle= tf::getYaw(last_map_pose.pose.orientation) - CV_PI/2;
	Point2f pose(last_map_pose.pose.position.x,last_map_pose.pose.position.y);
	Vec2i center= pointToGridCoord(pose);
	//may need to flip center coord order
	Ellipse(ROI,Point2i(center[0],center[1]),Size(3/gridRes,3/gridRes),angle,-CV_PI / 2,CV_PI/2,Scalar(1),-1);
	return ROI;
}
SparseMat_<char> getCameraClearROI(){
	/*
		return trapezoid covering camera view
	*/
}
SparseMat_<char> getLIDARROI(){
	SparseMat_<char> ROI = SparseMat(gridSize,gridSize,CV_8SC1);
	//may need to offset angles by some reference angle
	double angle= tf::getYaw(last_map_pose.pose.orientation) - CV_PI/2;
	Point2f pose(last_map_pose.pose.position.x,last_map_pose.pose.position.y);
	Vec2i center= pointToGridCoord(pose);
	//may need to flip center coord order
	Ellipse(ROI,Point2i(center[0],center[1]),Size(10/gridRes,10/gridRes),angle,-CV_PI / 2,CV_PI/2,Scalar(1),-1);
}
SparseMat_<char> getCameraROI(){
	return getCameraClearROI();
}

void copyPoints(sensor_msgs::PointCloud scanCloud){
	SparseMat_<char> cloud = SparseMat(gridSize,gridSize,CV_8SC1);
	vector<Point2f> points = pointCloudToPoints(scan_cloud);

	for(int i=0;i<points.size();i++){
		if(gridBounds.contains(points[i])){
			cloud(pointToGridIndex(points[i]))=1;
		}
	}
	dilate(cloud,cloud,fattener);	//fatten

	SparseMat_<char> ROI = using_camera? getCameraROI():getLIDARROI();
	SparseMat_<char> update;
	multiply(ROI,cloud,update,FILL_RATE);	//mask fattened points to keep out lies

	SparseMat_<char> clearROI = using_camera? getCameraClearROI():getLIDARClearROI();
	
	//may need to scale; this assumes results of comparison are 0,1 logic levels
	SparseMat_<char> clear = (clearROI > update) * -CLEAR_RATE; //avoid clearing pings
	update = update + clear;

	accumulateWeighted(update,cSpaceMat,DECAY_RATE,update);	//might need (1-DECAY_RATE)
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
		copyPoints(scanCloud);
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
	ros::init(argc,argv,"mapper");//name of this node

	tfl = new tf::TransformListener();
	ros::NodeHandle n;

	// Load parameters from server
	if (n.getParam("/mapper/loopRate", loopRate)){
		ROS_INFO("Mapper: loaded loopRate=%f",loopRate);
	} else{
		ROS_INFO("Mapper: error loading loopRate");
	}

	cout<<"2\n";
	ros::Rate loopTimer(loopRate); //will perform sleeps to enforce loop rate of "10" Hz
	while (ros::ok()&&!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce();
	cout<<"2\n";


	ros::Subscriber S1 = n.subscribe<sensor_msgs::PointCloud>("LIDAR_Cloud", 20, cloudCallback);
	ros::Subscriber S2 = n.subscribe<nav_msgs::Odometry>("odom", 10, odomCallback);
	ros::Publisher P = n.advertise<nav_msgs::OccupancyGrid>("CSpace_Map", 10);
	cout<<"2\n";
//	namedWindow("cSpace",CV_WINDOW_NORMAL);
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

