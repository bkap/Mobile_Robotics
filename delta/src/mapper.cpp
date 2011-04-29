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
#include "cvFuncs.h"

#define FILL_RATE 25
#define CLEAR_RATE 5
#define CAMERA_ROI_FILE "cameraROI_base_link"


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

Mat_<Point2f> cameraROICorners(2,2,Point2f(0,0));

const float gridWidth= 45;	//meters
const float gridRes = 0.05;	//meters per pixel
const float fattening=0.35;	//meters dilation of obstacles
const int fixedPoints = 1;	//fractional bits for locations on grid
const Point_<float> gridOrigin(-15,0); //grid x0,y0 in meters

const Size_<float> gridSize(gridWidth,gridWidth);	//meter width of grid
const Size_<int> gridMatSize(ceil(gridWidth/gridRes),ceil(gridWidth/gridRes));
const Rect_<float> gridBounds(gridOrigin,gridSize);
const Rect gridMatBounds(Point(0,0),Size(gridMatSize.width,gridMatSize.width));
const Scalar_<char> fillColor(FILL_RATE);
const Scalar_<char> clearColor(CLEAR_RATE);
Mat_<char> gridMat;
Mat_<uchar> lidarROI(gridMatSize,CV_8U);		//mask containing current area within LIDAR pings;; just a fattened raytrace
Mat_<uchar> cameraROI(gridMatSize,CV_8U);	//mask containing current viewable area
Mat_<uchar> cameraGrid(gridMatSize,CV_8U);	//occupancy grid based solely on camera
Mat_<uchar> LIDARGrid(gridMatSize,CV_8U);	//occupancy grid based solely on lidar

ros::Publisher *P,*Plid,*Pcam;
nav_msgs::OccupancyGrid grid,grid_cam,grid_lid;
bool init = false;

/*
	In accordance with OccupancyGrid, columns correspond to x and rows correspond to y
*/

//initializes the CSpace grid.
template <typename T>
void readMat(cv::Mat_<T>& mat, char* file){
	ifstream* infile = new ifstream(file,ifstream::in|ifstream::binary);
	int rows = 0,cols = 0,type=0,size=0;
	(*infile)>>rows;
	(*infile)>>cols;
	(*infile)>>type;
	(*infile)>>size;
	char* data = new char[size];
	infile->read(data,size);
	infile->close();

	int sizes[2] = {rows,cols};
	Mat_<T> temp = Mat(2,sizes,type,data);
	temp.copyTo(mat);
	delete[] data;
}

Mat_<uchar> patchInit(){
	Mat_<uchar> patch(2 * fattening / gridRes, 2 * fattening / gridRes, (uchar) 0);
	circle(patch,Point(patch.size().width,patch.size().width), fattening / gridRes - 3, FILL_RATE, -1, 8,1);
	GaussianBlur(patch,patch,Size(3,3),0,0);
	return patch;
}

void cSpaceInit()
{

	ROS_INFO("initializing mapper");
	grid.header.seq = 0;
	grid.header.frame_id = "map";
	//Output.header.stamp = time(NULL);
	grid.info.resolution = gridRes;
	//Output.info.map_load_time = time(NULL);
	grid.info.width = gridSize.width/gridRes;//NUM_WIDTH;
	grid.info.height = gridSize.height/gridRes;//NUM_HEIGHT;
	grid.info.origin.position.x = gridOrigin.x;//InitX-GRID_WIDTH/2.0;
	grid.info.origin.position.y = gridOrigin.y;//InitY-GRID_HEIGHT/2.0;
	grid.info.origin.position.z = 0;
	grid.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
	grid_lid.info = grid.info;
	grid_cam.info = grid.info;
	grid_lid.header.frame_id="map";
	grid_cam.header.frame_id="map";
	grid_cam.header.seq = 0;
	grid_lid.header.seq = 0;
	vector<char>* data = new vector<char>((grid.info.width) * (grid.info.height));
	vector<char>* data1 = new vector<char>((grid.info.width) * (grid.info.height));
	vector<char>* data2 = new vector<char>((grid.info.width) * (grid.info.height));
	ROS_INFO("created occupancy grid with %d x %d elements",(grid.info.width),(grid.info.height));
	
	gridMat = Mat::zeros(gridMatSize.height,gridMatSize.width,CV_8S);
	cameraGrid = Mat::zeros(gridMatSize.height,gridMatSize.width,CV_8U);
	LIDARGrid  = Mat::zeros(gridMatSize.height,gridMatSize.width,CV_8U);
	grid.data.assign(data->begin(),data->end());	
	grid_cam.data.assign(data1->begin(),data1->end());	
	grid_lid.data.assign(data2->begin(),data2->end());
	gridMat.data = (uchar *) &grid.data[0];		//kind of shifty, but cturtle demands it.
	cameraGrid.data=(uchar *)&grid_cam.data[0];
	LIDARGrid.data=(uchar *)&grid_lid.data[0];
	
	gridMat = 0;
	//LIDARGrid = 128;
	//cameraGrid = 128;
	last_map_pose.pose.position.x = 0;
	last_map_pose.pose.position.y = 0;
	last_map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	
	//ROS_INFO("created cSpace grid with %d elements",gridMat.rows*gridMat.cols);
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
void drawHit(Mat_<uchar>& grid, Point2f hit){
	static int radius = static_cast<int>(fattening / gridRes * (1 << fixedPoints));
	static Mat_<uchar> patch = patchInit();
	//Point center = pointToGridPoint(hit,fixedPoints);
	//circle(gridMat, center, radius, fillColor, -1, 8,fixedPoints);
	Point center = pointToGridPoint(hit);	//roi doesn't support fixedpoint
	Rect roi_ = Rect(center.x,center.y,radius,radius) - Point(radius,radius);	//rect centered on hit
	Rect roi = roi_ & gridMatBounds;
	if(roi.size() != roi_.size()){
		//ROS_INFO("roi doesnt fit: %i,%i : %i,%i",roi.width,roi.height,roi_.width,roi_.height);
		return;
	}
	Mat_<uchar> t(grid,roi);
	if(t.size()!= patch.size()){
		//ROS_INFO("hit doesnt fit: %i,%i : %i,%i",t.rows,t.cols,patch.rows,patch.cols);
		return;
	}
	//ROS_INFO("drawing a hit");
	t+= patch;
}

/*
	Given a grid, a pointcloud in map frame, and a mask for which points to accept,
	renders the points onto the grid, subject to the mask
*/
void addHits(Mat_<uchar>& grid, const sensor_msgs::PointCloud& cloud, vector<bool> mask= vector<bool>()){
	bool maskall = mask.size() == 0;
	vector<Point3f> points;
	ROS2CVPointCloud(cloud,points);
	Point2f hit;
	for(unsigned int i=0;i<points.size();i++){
		hit.x = points[i].x;
		hit.y = points[i].y;
		if(maskall || mask[i]){
			//ROS_INFO("adding hit at (%f,%f)",hit.x,hit.y);
			drawHit(grid,hit);
		}
	}
}
void updateGrid(){

	ROS_INFO("updating grid");

        for(int i =0; i<cameraGrid.rows; i++)
	{
		//cout<<"row "<<i<<"\n";
		for (int j = 0; j<cameraGrid.cols; j++)
		{

			//cout<<" col "<<j<<"\n"; 
                	grid.data[i*cameraGrid.cols+j]= LIDARGrid(i,j)>cameraGrid(i,j)?(char)(LIDARGrid(i,j)) : (char)(cameraGrid(i,j));//-128;
		}
        }


	//cvNamedWindow("grid",CV_WINDOW_AUTOSIZE);
	//imshow("grid",cameraGrid);
	//waitKey(2);


		P->publish(grid);
		Pcam->publish(grid_cam);
		Plid->publish(grid_lid);
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
		mask.push_back(gridBounds.contains(hit) && gridMatBounds.contains(cell) && cameraROI(cell)!=0 && lidarROI(cell)!=0);
		//if(!gridBounds.contains(hit))	ROS_INFO("hit off map");
		//if(!cameraROI(cell))	ROS_INFO("hit not viewable");
		//if(!lidarROI(cell))	ROS_INFO("hit not in lidar");
		//if(gridBounds.contains(hit) && gridMatBounds.contains(cell) && cameraROI(cell)!=0 && lidarROI(cell)!=0) ROS_INFO("hit accepted");
	}
}
/*
	
*/
void cameraCallback(const sensor_msgs::PointCloud::ConstPtr& scan_cloud) 
{
	return;
	if (!init){
		ROS_INFO("skipping camera until initialization");
		return;
	}
	ROS_INFO("MAPPER found %d points",scan_cloud->points.size());
	ROS_INFO("camera callback");
	subtract(cameraGrid,clearColor,cameraGrid,cameraROI);
	//ROS_INFO("cleared ROI");
	vector<bool> mask;
	maskCamera(*scan_cloud,mask);
	addHits(cameraGrid,*scan_cloud,mask);
	updateGrid();
}

/*
	returns a point representing the vector wrt the given pose
*/
Point2f relativeTo(Point2f point, geometry_msgs::Pose& pose){
	Vec2f refDir,refPos,rDir,newDir;
	ROS2CVPose(pose,refPos,refDir);

	Vec2f r(point.x - refPos[0],point.y - refPos[1]);// = point - refPos;
	getUnitVec(rDir,angle(r));
	getUnitVec(newDir,angle(r)+angle(refDir));
	Vec2f dest = newDir * (float) norm(r);
	dest = dest + refPos;
	return Point2f(dest[0],dest[1]);
}



/*
	Given a PointCloud of lidar pings in map frame,
	raytraces pings to generate a mask for clear space
*/
void updateLIDARROI(sensor_msgs::PointCloud cloud){
	lidarROI = (uchar)0;
	vector<Point3f> points;
	ROS2CVPointCloud(cloud,points);
	Point2f hit;
	Point2f robot(last_map_pose.pose.position.x,last_map_pose.pose.position.y);
	for(unsigned int i=0;i<points.size();i++){
		hit.x = points[i].x;
		hit.y = points[i].y;
		line(lidarROI, pointToGridPoint(robot,fixedPoints), pointToGridPoint(hit,fixedPoints), 255, 2, 8, fixedPoints);
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
				**it = saturate_cast<uchar>((uchar) **it - (uchar)CLEAR_RATE);
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
		//if(!gridBounds.contains(hit))
			//ROS_INFO("hit at (%f,%f) out of bounds",hit.x,hit.y);
		//if(r>=400)
			//ROS_INFO("hit is %f from robot",r);
		//if(gridBounds.contains(hit) && r < 400)
			//ROS_INFO("hit at (%f,%f) accepted",hit.x,hit.y);
	}
}
geometry_msgs::PoseStamped temp;
tf::TransformListener *tfl;
/*
	adds new scan to lidar grid
*/
void lidarCallback(const sensor_msgs::PointCloud::ConstPtr& scan_cloud_) 
{

	static sensor_msgs::PointCloud scan_cloud;
	tfl->transformPointCloud("map", *(scan_cloud_), scan_cloud);
	if (!init)	return;
	//return;
	ROS_INFO("LIDAR callback");
	updateLIDARROI(scan_cloud);
	//return;
	vector<bool> mask;
	maskLIDAR(scan_cloud, mask);
	clearLIDAR(scan_cloud,mask);
	//ROS_INFO("adding hit");
	addHits(LIDARGrid,scan_cloud,mask);
	updateGrid();

}





/*
	tracks robot location and triggers updates of cameraROI when robot moves
	initializes various things on first call
*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) 
{
	//ROS_INFO("odom callback");
	last_odom = *odom;
        temp.pose = last_odom.pose.pose;
        temp.header = last_odom.header;
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
}


/* takes roi corners in map frame and sets roi mask */
void cameraROICallback(const sensor_msgs::PointCloud::ConstPtr& cloud){
	/*	convert to vector<Point>	*/
	vector<Point3f> points;
	ROS2CVPointCloud(*cloud,points);
	vector<Point> corners;
	for(int i=0;i<4;i++){
		corners.push_back(pointToGridPoint(Point2f(points[i].x,points[i].y),fixedPoints));
	}

	/*	update roi	*/
	cameraROI = (uchar) 0;
	fillConvexPoly(cameraROI,&(corners.front()),4,(uchar) 255,8,fixedPoints);
}



int main(int argc,char **argv)
{
	cout<<"2\n";
	ros::init(argc,argv,"mapper");//name of this node

	tfl = new tf::TransformListener();
	ros::NodeHandle n;
/*
	// Load parameters from server
	if (n.getParam("/mapper/loopRate", loopRate)){
		ROS_INFO("Mapper: loaded loopRate=%f",loopRate);
	} else{
		ROS_INFO("Mapper: error loading loopRate");
	}
*/
	cout<<"2\n";
	ros::Rate loopTimer(loopRate); //will perform sleeps to enforce loop rate of "10" Hz
	while (ros::ok()&&!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce();
	cout<<"2\n";

	P = new ros::Publisher();
	Pcam = new ros::Publisher();
	Plid = new ros::Publisher();
	(*P) = n.advertise<nav_msgs::OccupancyGrid>("CSpace_Map", 10);
	(*Pcam) = n.advertise<nav_msgs::OccupancyGrid>("cam_Map", 10);
	(*Plid) = n.advertise<nav_msgs::OccupancyGrid>("lid_Map", 10);
	ros::Subscriber S1 = n.subscribe<sensor_msgs::PointCloud>("LIDAR_Cloud", 20, lidarCallback);
	ros::Subscriber S2 = n.subscribe<nav_msgs::Odometry>("odom", 10, odomCallback);
	ros::Subscriber S3 = n.subscribe<sensor_msgs::PointCloud>("Camera_Cloud",5,cameraCallback);
	ros::Subscriber S4 = n.subscribe<sensor_msgs::PointCloud>("Camera_view",10,cameraROICallback);


//	P->publish(grid);

	cout<<"2\n";
//	namedWindow("cSpace",CV_WINDOW_NORMAL);
	while(ros::ok())
	{
		ros::spin();
	}
	ROS_INFO("Mapper terminated");
	return 0;
}

