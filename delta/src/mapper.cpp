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
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <fstream>
#include <iostream>
#include <cvFuncs.h>

#define FILL_RATE 25
#define CLEAR_RATE 5
#define CAMERA_ROIx_FILE "delta/cameraROIx_base_link"
#define CAMERA_ROIy_FILE "delta/cameraROIy_base_link"


using namespace std;
using namespace cv;

/*
	Separated lidar and camera occupancy grids
	implemented lidar clearing via raytracing
	implemented camera clearing via viewable area masking

	mask camera by lidar pings to prevent addition of poorly-mapped camera points

*/

/*TODO

finish and verify the rewrite
	Clearing is presently mediocre
		cameara clears via subtraction, lidar clears via setting

	convert from fixed origin to rolling window

various fixes needed:
	use real transforms
	eliminate extraneous computation


camera mask depends on lidar mask:
	valid camera points exist only behind valid pings
*/

double loopRate = 10;

sensor_msgs::PointCloud scanCloud;
geometry_msgs::PoseStamped last_map_pose;
nav_msgs::Odometry last_odom;

Mat_<Vec2f> cameraROICorners(2,2,Vec2f(0,0));

const float gridWidth= 45;	//meters
const float gridRes = 0.05;	//meters per pixel
const float fattening=0.35;	//meters dilation of obstacles
const int fixedPoints = 1;	//fractional bits for locations on grid
const Point_<float> gridOrigin(-10,-10); //grid x0,y0 in meters

const Size_<float> gridSize(gridWidth,gridWidth);	//meter width of grid
const Size_<int> gridMatSize(ceil(gridWidth/gridRes),ceil(gridWidth/gridRes));
const Rect_<float> gridBounds(gridOrigin,gridSize);
const Scalar_<char> fillColor(FILL_RATE);
const Scalar_<char> clearColor(CLEAR_RATE);
Mat_<char> gridMat;
Mat_<char> lidarROI(gridMatSize,CV_8S);		//mask containing current area within LIDAR pings;; just a fattened raytrace
Mat_<char> cameraROI(gridMatSize,CV_8S);	//mask containing current viewable area
Mat_<char> cameraGrid(gridMatSize,CV_8S);	//occupancy grid based solely on camera
Mat_<char> LIDARGrid(gridMatSize,CV_8S);	//occupancy grid based solely on lidar


nav_msgs::OccupancyGrid grid;
bool init = false;
/*
	In accordance with OccupancyGrid, columns correspond to x and rows correspond to y
*/

//initializes the CSpace grid.



template <typename T>
void readMat(cv::Mat_<T>& mat, char* file){
	ifstream* infile = new ifstream(file,ifstream::in&ifstream::binary);
	int rows = 0,cols = 0;
	(*infile)>>rows;
	(*infile)>>cols;
	mat = Mat_<T>(rows,cols);
	T val;
	for(int i=0;i<mat.rows;i++){
		for(int j=0;j<mat.cols;j++){
			(*infile)>>val;
			mat(i,j)=val;
		}
	}
	infile->close();
}
template <typename T>
void writeMat(cv::Mat_<T>& mat, char* file){
	ofstream* ofile = new ofstream(file,ofstream::out&ofstream::binary);
	(*ofile)<<mat.rows<<" ";
	(*ofile)<<mat.cols<<" ";
	
	for(int i = 0;i<mat.rows;i++){
		for(int j = 0;j<mat.cols;j++){
			(*ofile)<<(T) mat(i,j)<<" ";
		}
	}
	ofile->close();
}



void cSpaceInit()
{
	cout<<"creating cSpace grid:"<<endl;
	grid.header.seq = 0;
	grid.header.frame_id = "map";
	//Output.header.stamp = time(NULL);
	grid.info.resolution = gridRes;
	//Output.info.map_load_time = time(NULL);
	grid.info.width = gridSize.width;//NUM_WIDTH;
	grid.info.height = gridSize.height;//NUM_HEIGHT;
	grid.info.origin.position.x = gridOrigin.x;//InitX-GRID_WIDTH/2.0;
	grid.info.origin.position.y = gridOrigin.y;//InitY-GRID_HEIGHT/2.0;
	grid.info.origin.position.z = 0;
	grid.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
	vector<char>* data = new vector<char>((grid.info.width) * (grid.info.height));
	grid.data.assign(data->begin(),data->end()); //I realize that this size should be 8 by definition, but this is good practice.	
	gridMat = cv::Mat(grid.data,false);
	gridMat = gridMat.reshape(gridMatSize.width);

	Mat_<float> x(2,2,0.f);
	Mat_<float> y(2,2,0.f);

	
	readMat<float>(x,CAMERA_ROIx_FILE);
	readMat<float>(y,CAMERA_ROIy_FILE);

	for (int i=0;i<4;i++){
		cameraROICorners(i) = Vec2f(x(i), y(i));
	}

	cout<<"\tcreated cSpace grid with "<<gridMat.total()<<" elements"<<endl;
}

//return index in grid corresponding to x,y coordinate in frame
int pointToGridIndex(Point2f point){
	int gx = round((point.x - gridOrigin.x)/gridRes);
	int gy = round((point.y - gridOrigin.y)/gridRes);
	return gx + gridMatSize.width * gy;
}

Point pointToGridPoint(Point2f point, size_t shift = 0){
	float gx = (point.x - gridOrigin.x)/gridRes;
	float gy = (point.y - gridOrigin.y)/gridRes;

	if(shift==0)	return Point(round(gx),round(gy));

	float s = (1 << shift);
	int gxs = static_cast<int>(gx*s);
	int gys = static_cast<int>(gy*s);
	return Point(gxs,gys);	
}
void drawHit(Mat_<char>& grid, Point2f hit){
	static int radius = static_cast<int>(fattening / gridRes * (1 << fixedPoints));
	Point center = pointToGridPoint(hit,fixedPoints);

	circle(gridMat, center, radius, fillColor, -1, 8,fixedPoints);
}

/*
	Given a grid, a pointcloud in map frame, and a mask for which points to accept,
	renders the points onto the grid, subject to the mask
*/
void addHits(Mat_<char>& grid, const sensor_msgs::PointCloud& cloud, vector<bool> mask= vector<bool>()){
	bool maskall = mask.size() == 0;
	vector<Point3f> points;
	ROS2CVPointCloud(cloud,points);
	Point2f hit;
	for(unsigned int i=0;i<points.size();i++){
		hit.x = points[i].x;
		hit.y = points[i].y;
		if(maskall || mask[i]){
			drawHit(grid,hit);
		}
	}
}
void updateGrid(){
	cv::MatIterator_<char> it=gridMat.begin(), it_end = gridMat.end();
	cv::MatIterator_<char> camerait=cameraGrid.begin(), camerait_end = cameraGrid.end();
	cv::MatIterator_<char> lidarit=LIDARGrid.begin(), lidarit_end = LIDARGrid.end();
	
        for(;it!=it_end;++it,++camerait,++lidarit){
                *it = (*lidarit) > (*camerait)? *lidarit : *camerait;
        }
}

/*
	Given a point cloud in the map frame,
	returns a mask for which are located within a valid region of the grid:
		in presently viewable camera area and not behind a lidar ping
*/
void maskCamera(const sensor_msgs::PointCloud& cloud, vector<bool>& mask){
	mask.clear();
	vector<Point3f> points;
	ROS2CVPointCloud(cloud,points);
	Point2f hit;
	for(unsigned int i=0;i<points.size();i++){
		hit.x = points[i].x;
		hit.y = points[i].y;
		Point cell = pointToGridPoint(hit);
		mask.push_back(gridBounds.contains(hit) && (cameraROI(cell) & lidarROI(cell)));
	}
}
/*
	
*/
void cameraCallback(const sensor_msgs::PointCloud::ConstPtr& scan_cloud) 
{
	if (!init)	return;
	//clearCamera();
	subtract(cameraGrid,clearColor,cameraGrid,cameraROI);
	vector<bool> mask;
	maskCamera(*scan_cloud,mask);
	addHits(cameraGrid,*scan_cloud,mask);
	updateGrid();
}

/*
	returns a point representing the vector wrt the given pose
*/
Point2f relativeTo(Vec2f point, geometry_msgs::Pose& pose){
	Vec2f refDir,refPos,rDir,newDir;
	ROS2CVPose(pose,refPos,refDir);

	Vec2f r = point - refPos;//(point.x - refPos[0],point.y - refPos[1]);
	getUnitVec(rDir,angle(r));
	getUnitVec(newDir,angle(r)+angle(refDir));
	Vec2f dest = newDir * (float) norm(r);
	dest = dest + refPos;
	return Point2f(dest[0],dest[1]);
}

/*
	updates cameraROI to reflect presently viewable camera area
*/
void updateCameraROI(){
	static vector<Point> corners;
	if(corners.size()!=0){
		//may be bad pointer
		cameraROI = (char) 0;
		corners.clear();	
	}
	corners.push_back(pointToGridPoint(relativeTo(cameraROICorners(0,0),last_map_pose.pose),fixedPoints));
	corners.push_back(pointToGridPoint(relativeTo(cameraROICorners(0,1),last_map_pose.pose),fixedPoints));
	corners.push_back(pointToGridPoint(relativeTo(cameraROICorners(1,1),last_map_pose.pose),fixedPoints));
	corners.push_back(pointToGridPoint(relativeTo(cameraROICorners(1,0),last_map_pose.pose),fixedPoints));
	//may be bad pointer	
	fillConvexPoly(cameraROI,&(corners.front()),4,Scalar_<char>(1),8,fixedPoints);
}

/*
	Given a PointCloud of lidar pings in map frame,
	raytraces pings to generate a mask for clear space
*/
void updateLIDARROI(sensor_msgs::PointCloud cloud){
	lidarROI = (char)0;
	vector<Point3f> points;
	ROS2CVPointCloud(cloud,points);
	Point2f hit;
	Point2f robot(last_map_pose.pose.position.x,last_map_pose.pose.position.y);
	for(unsigned int i=0;i<points.size();i++){
		hit.x = points[i].x;
		hit.y = points[i].y;
		line(lidarROI, pointToGridPoint(robot,fixedPoints), pointToGridPoint(hit,fixedPoints), Scalar(127), 2, 8, fixedPoints);
	}
}

/*
	Given a PointCloud of lidar pings in map frame,
	clears grid by raytracing out to pings
*/
void clearLIDAR(const sensor_msgs::PointCloud& cloud, vector<bool> mask = vector<bool>()){
	bool maskall = mask.size()==0;	
	vector<Point3f> points;
	ROS2CVPointCloud(cloud,points);
	Point2f hit;
	Point2f robot(last_map_pose.pose.position.x,last_map_pose.pose.position.y);
	for(unsigned int i=0;i<points.size();i++){
		hit.x = points[i].x;
		hit.y = points[i].y;
		if(maskall || mask[i]){
			//line(LIDARGrid, pointToGridPoint(robot,fixedPoints), pointToGridPoint(hit,fixedPoints), Scalar(-CLEAR_RATE), 1, 8, fixedPoints);
			LineIterator it(LIDARGrid,pointToGridPoint(robot),pointToGridPoint(hit));
			for(int j=0;j<it.count;j++, ++it){
				**it = saturate_cast<char>((char) **it - (char)CLEAR_RATE);
			}
		}
	}
}
/*
	Given a point cloud in the map frame,
	returns a mask for which are located within a valid region of the grid:
		within 20m of robot
		in bounds of occupancy grid
*/
void maskLIDAR(const sensor_msgs::PointCloud& cloud, vector<bool>& mask){
	mask.clear();
	vector<Point3f> points;
	ROS2CVPointCloud(cloud,points);
	Point2f hit;
	Point2f robot(last_map_pose.pose.position.x,last_map_pose.pose.position.y);
	for(unsigned int i=0;i<points.size();i++){
		hit.x = points[i].x;
		hit.y = points[i].y;
		float r = (hit.x-robot.x)*(hit.x-robot.x)+(hit.y-robot.y)*(hit.y-robot.y);
		mask.push_back(gridBounds.contains(hit) && r < 400);
	}
}

/*
	adds new scan to lidar grid
*/
void lidarCallback(const sensor_msgs::PointCloud::ConstPtr& scan_cloud) 
{
	if (!init)	return;
	vector<bool> mask;
	maskLIDAR(*scan_cloud, mask);
	clearLIDAR(*scan_cloud,mask);
	addHits(LIDARGrid,*scan_cloud,mask);
}



geometry_msgs::PoseStamped temp;
tf::TransformListener *tfl;

/*
	tracks robot location and triggers updates of cameraROI when robot moves
	initializes various things on first call
*/
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
	if(init == false){
		cout<<"odom callback initialized"<<endl;
		cSpaceInit();
		init = true;
	}
	updateCameraROI();
	
}

int main(int argc,char **argv)
{
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

	ros::Subscriber S1 = n.subscribe<sensor_msgs::PointCloud>("LIDAR_Cloud", 20, lidarCallback);
	ros::Subscriber S2 = n.subscribe<nav_msgs::Odometry>("odom", 10, odomCallback);
	ros::Subscriber S3 = n.subscribe<sensor_msgs::PointCloud>("Camera_Cloud",1,cameraCallback);
	ros::Publisher P = n.advertise<nav_msgs::OccupancyGrid>("CSpace_Map", 10);

	cout<<"2\n";
//	namedWindow("cSpace",CV_WINDOW_NORMAL);
	while(ros::ok())
	{
		ros::spinOnce();
		P.publish(grid);
		loopTimer.sleep();
	}
	
	return 0;
}

