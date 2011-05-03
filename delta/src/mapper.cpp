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
#include <algorithm>
#include "cvFuncs.h"

#define ADDITIVE 1

#ifdef ADDITIVE
#define FILL_RATE 25
#define CLEAR_RATE 5
#else
#define ADDITIVE 0
#define FILL_RATE 255
#define CLEAR_RATE 0
#endif


using namespace std;
using namespace cv;

/*	Connor Balin

	The mapper tracks map occupancy based on sensor readings, incorporating LIDAR and camera information.
	Each sensor has an individual occupancy map, which is published for debugging and visualization.
	Finite persistence is implemented individually for each sensor, with LIDAR clearing implemented by raytracing and camera clearing implemented by adjusting the 		occupancy of the presently viewable trapezoid.

	Occupancies are given values in the range [-128,127], with negative numbers denoting probable availability and positive numbers denoting probable obstacles.

	To prevent poor localization of tall things by the camera, camera points are masked by the area most recently swept by LIDAR raytracing to eliminate non-ground-plane

	Merging of the individual sensor grids into the primary occupancy map is performed by taking the maximum occupancy recorded by any sensor for each cell.

	Two modes of occupancy calculation are supported.  In the additive mode, sensor readings increment the surrounding cells in the map.  In the setting mode, the 		sensor readings strictly write over the occupancy grid, merging (again with max) the cells surrounding the sensor reading with a gaussian blob.  This results in 		smooth edges to the occupancy map, with values decaying with distance from nearest obstacle and allows the planner to favor paths far away from obstacles.

*/

double loopRate = 10;		//10 Hz loop

sensor_msgs::PointCloud scanCloud;
geometry_msgs::PoseStamped last_map_pose;
nav_msgs::Odometry last_odom;

const float gridWidth= 45;			//meters
const float gridRes = 0.05;			//meters per pixel
const float fattening= ADDITIVE? 0.25:.45;	//meters dilation of obstacles
const int fixedPoints = 1;			//fractional bits for locations on grid
const Point_<float> gridOrigin(-15,0); 		//grid x0,y0 in meters

const Size_<float> gridSize(gridWidth,gridWidth);	//meter width of grid
const Size_<int> gridMatSize(ceil(gridWidth/gridRes),ceil(gridWidth/gridRes)); //pixel dimensions of occupancy maps
const Rect_<float> gridBounds(gridOrigin,gridSize);				//map frame bounding rectangle of occupancy maps
const Rect gridMatBounds(Point(0,0),Size(gridMatSize.width,gridMatSize.width));	//pixel frame bounding rectangle of occupancy maps

Mat_<char> gridMat;				//opencv Mat wrapping the OccupanyGrid for convenience
Mat_<uchar> lidarROI(gridMatSize,CV_8U);	//mask containing current area within LIDAR pings;; just a fattened raytrace
Mat_<uchar> cameraROI(gridMatSize,CV_8U);	//mask containing current area viewable by camera
Mat_<uchar> cameraGrid(gridMatSize,CV_8U);	//occupancy grid based solely on camera
Mat_<uchar> LIDARGrid(gridMatSize,CV_8U);	//occupancy grid based solely on lidar

ros::Publisher *P,*Plid,*Pcam;
nav_msgs::OccupancyGrid grid,grid_cam,grid_lid;
bool init = false;

/* function to read Mats from a file */
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

/*  
	initializes the round bit shoved into the grid wherever a lidar ping/ camera hit happens.
	It is a circle of radius slightly less than the set fattening distance with gaussian blur applied to make of the difference.
	After blurring, the patch is normalized to a range of [0 , FILL_RATE]
*/
Mat_<uchar> patchInit(){
	Mat_<uchar> patch=Mat::zeros(2 * fattening / gridRes, 2 * fattening / gridRes, CV_8U);
	circle(patch,Point(patch.size().width/2.0,patch.size().width/2.0), fattening / gridRes - 2, FILL_RATE, -1, 8,1);

	if(ADDITIVE)	GaussianBlur(patch,patch,Size(5,5),0,0);
	else		GaussianBlur(patch,patch,Size(11,11),5,5);

	Mat_<uchar> patch_;
	normalize(patch,patch_, 0, FILL_RATE, NORM_MINMAX);
	return patch_;
}

/* 
	Initializes the Occupancy grids for each sensor and the main OccupancyGrid and wraps them all in cv::Mats
*/
void cSpaceInit(){

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
	gridMat.data = (uchar *) &grid.data[0];		//dirty pointer hack
	cameraGrid.data=(uchar *)&grid_cam.data[0];
	LIDARGrid.data=(uchar *)&grid_lid.data[0];
	
	gridMat = 0;
	if(ADDITIVE){
		LIDARGrid = 128;
		cameraGrid = 128;
	}
	last_map_pose.pose.position.x = 0;
	last_map_pose.pose.position.y = 0;
	last_map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
}

/*
	Given a Point in map co-ordinates,
	returns corresponding Point in OccupancyGrid index co-ordinates
	to specified fixed point precision 
*/
Point pointToGridPoint(Point2f point, size_t shift = 0){
	float gx = (point.x - gridOrigin.x)/gridRes;
	float gy = (point.y - gridOrigin.y)/gridRes;

	if(shift==0)	return Point(round(gx),round(gy));
	
	float s = (1 << shift);
	int gxs = static_cast<int>(gx*s);
	int gys = static_cast<int>(gy*s);
	return Point(gxs,gys);	
}

/*
	Given a sensor grid to draw on and a map co-ordinate locating the hit,
	
	draws the patch centered at the proper grid index
	if ADDITIVE is #defined, the gaussian patch is added to the occupancy grid
	if ADDITIVE isnt #defined, the occupancy grid is set to the value in the patch unless it already has a higher value
*/
void drawHit(Mat_<uchar>& grid, Point2f hit){
	static int radius = static_cast<int>(fattening / gridRes * (1 << fixedPoints));
	static Mat_<uchar> patch = patchInit();
	
	
	Point center = pointToGridPoint(hit);	//roi doesn't support subpixel precision
	Rect roi_ = Rect(center.x-radius/2.0,center.y-radius/2.0, radius,radius);	//rect bounding patch and centered on hit
	Rect roi = roi_ & gridMatBounds;	//intersection of roi and map boundary
	if(roi.size() != roi_.size())	return;	//do not draw patches if they don't entirely fit on the map because it is inconvenient to do so

	Mat_<uchar> t(grid,roi);
	if(t.size()!= patch.size())	return; //do not draw patches if they don't entirely fit on the map because it is inconvenient to do so
	//ROS_INFO("drawing a hit");	//don't print this out because it is slow
	if(ADDITIVE)	t+=patch;	
	else		t= max(t, patch);
}

/*
	Given a grid, a pointcloud in map frame, and an optional mask to filter points,
	renders the points onto the grid
*/
void addHits(Mat_<uchar>& grid, const sensor_msgs::PointCloud& cloud, vector<bool> mask= vector<bool>()){
	bool maskall = mask.size() == 0;	//if no mask is provided,  accept all points in the cloud
	vector<Point3f> points;			
	ROS2CVPointCloud(cloud,points);		//convert the point cloud into a vector<Point3f>
	Point2f hit;				
	for(unsigned int i=0;i<points.size();i++){
		hit.x = points[i].x;
		hit.y = points[i].y;
		if(maskall || mask[i]){
			drawHit(grid,hit);	//draw hits that pass the mask
		}
	}
}

/*
	merges the lidar and camera grid into the main OccupancyGrid by placing the max value from either individual grid then shifting to convert from uchar to char
	also publishes all three occupancy grids
*/
void updateGrid(){

       for(int i =0; i<cameraGrid.rows; i++){
		for (int j = 0; j<cameraGrid.cols; j++){

			uchar lidar = LIDARGrid(i,j);
			uchar cam = cameraGrid(i,j);
			int max;

			if (lidar>cam) 	max = lidar;
			else 		max = cam;
		
			gridMat.at<char>(i,j) = saturate_cast<char>(max-128);
		}
        }
/*	debug viewing of the merged mat since RVIZ lies when it colors things.
	cvNamedWindow("Occupancy grid",CV_WINDOW_AUTOSIZE);	
	imshow("Occupancy grid",gridMat);
	waitKey(2);
*/
	P->publish(grid);
	Pcam->publish(grid_cam);
	Plid->publish(grid_lid);
}

/*
	Given a point cloud in the map frame,
		returns a mask rejecting all points in the cloud which do not lie within the most recently traced rays to LIDAR pings
		any points not in this region are probably not on the ground plane, so they will project poorly and should be thrown out
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
	}
}

/*
	Given a point cloud in the map frame corresponding to the edges of orange objects seen by the camera,
	clears the entire viewable area on the occupancy grid then fills patches along any edges

	if ADDITIVE is #defined, the clearing is performed by subtraction
	if ADDITIVE isnt#defined, the clearing is performed by zeroing the occupancy grid within the camera's viewable area

	calls updateGrid() to publish all the occupancy maps
*/
void cameraCallback(const sensor_msgs::PointCloud::ConstPtr& scan_cloud){
//	return;
	if (!init){
		ROS_INFO("skipping camera until initialization");
		return;
	}
	ROS_INFO("MAPPER found %d points",scan_cloud->points.size());
	ROS_INFO("camera callback");
	if(ADDITIVE)	subtract(cameraGrid,CLEAR_RATE,cameraGrid,cameraROI);
	else		bitwise_xor(cameraROI, cameraROI, cameraGrid, cameraROI);//shifty masking: cameraROI will be either 0 or 255, so this works out

	//ROS_INFO("cleared ROI");
	vector<bool> mask;
	maskCamera(*scan_cloud,mask);
	addHits(cameraGrid,*scan_cloud,mask);
	updateGrid();
}

/*
	Given a PointCloud of lidar pings in map frame,
	raytraces pings to generate a mask for ground-plane space for the camera
*/
void updateLIDARROI(sensor_msgs::PointCloud cloud){
	lidarROI = (uchar)0;		//zero out the old roi
	vector<Point3f> points;
	ROS2CVPointCloud(cloud,points);
	Point2f hit;
	Point2f robot(last_map_pose.pose.position.x,last_map_pose.pose.position.y);
	for(unsigned int i=0;i<points.size();i++){
		hit.x = points[i].x;
		hit.y = points[i].y;
		line(lidarROI, pointToGridPoint(robot,fixedPoints), pointToGridPoint(hit,fixedPoints), 255, 2, 8, fixedPoints);		//trace rays 2 pixels wide to pings
	}
}

/*
	Given a PointCloud of lidar pings in map frame and an optional mask to filter points,
	clears lidar grid by raytracing to pings that pass the mask

	if ADDITIVE is #defined, cells along the line are decremented by CLEAR_RATE
	if ADDITIVE isnt#defined, cells alond the line are set to CLEAR_RATE

	when ADDITIVE is #defined, rays are 1 pixel wide because LineIterators do not support line width,
 	otherwise, they are 2 cm wide to help clear space for planning.
*/
void clearLIDAR(const sensor_msgs::PointCloud& cloud, vector<bool> mask = vector<bool>()){
	bool maskall = mask.size()==0;	//if no mask is specified, accept all points in the cloud
	vector<Point3f> points;
	ROS2CVPointCloud(cloud,points);
	Point2f hit;
	Point2f robot(last_map_pose.pose.position.x,last_map_pose.pose.position.y);
	for(unsigned int i=0;i<points.size();i++){
		hit.x = points[i].x;
		hit.y = points[i].y;
		if(maskall || mask[i]){
			if(ADDITIVE){	//increment values along the ray
				LineIterator it(LIDARGrid,pointToGridPoint(robot),pointToGridPoint(hit));
				for(int j=0;j<it.count;j++, ++it){
					**it = saturate_cast<uchar>((uchar) **it - (uchar)CLEAR_RATE);
				}
			}else{	//set the ray to CLEAR_RATE
				line(LIDARGrid, pointToGridPoint(robot,fixedPoints), pointToGridPoint(hit,fixedPoints), CLEAR_RATE, 2, 8, fixedPoints);
			}
			
		}
	}
}
/*
	Given a point cloud in the map frame,
	returns a mask passing points located within a valid region of the grid:
		not an 80m ping
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
		mask.push_back(gridBounds.contains(hit) && r < 6375);
	}
}

geometry_msgs::PoseStamped temp;
tf::TransformListener *tfl;
/*
	Given a PointCloud, 
	raytrace to clear behind pings, draw pings on grid and update ground-plane mask	
*/
void lidarCallback(const sensor_msgs::PointCloud::ConstPtr& scan_cloud_){
	static sensor_msgs::PointCloud scan_cloud;
	tfl->transformPointCloud("map", *(scan_cloud_), scan_cloud);
	if (!init)	return;
	updateLIDARROI(scan_cloud);
	vector<bool> mask;
	maskLIDAR(scan_cloud, mask);
	clearLIDAR(scan_cloud,mask);
	addHits(LIDARGrid,scan_cloud,mask);

}

/*
	tracks robot location for ray tracing
	Also, calls all the initialization stuff on the first callback
*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){
	//ROS_INFO("odom callback");
	last_odom = *odom;
        temp.pose = last_odom.pose.pose;
        temp.header = last_odom.header;
        try{
          tfl->transformPose("map", temp, last_map_pose);
        } 
	catch (tf::TransformException ex){
	  cout << "We caught an error!" << endl;
          ROS_ERROR("%s", ex.what());
        }
	if(init == false){
		cout<<"odom callback initialized"<<endl;
		cSpaceInit();
		init = true;
	}
}


/*
	Given a PointCloud in map co-ordinates containing the 4 corners of the area currently viewable by the camera,
	updates the camera ROI to reflect the current viewable area

*/
void cameraROICallback(const sensor_msgs::PointCloud::ConstPtr& cloud){
	/*	convert to vector<Point>	*/
	vector<Point3f> points;
	ROS2CVPointCloud(*cloud,points);
	vector<Point> corners;
	for(int i=0;i<4;i++){
		corners.push_back(pointToGridPoint(Point2f(points[i].x,points[i].y),fixedPoints));
	}

	/*	update roi	*/
	cameraROI = (uchar) 0;		//zeros the old mask
	fillConvexPoly(cameraROI,&(corners.front()),4,(uchar) 255,8,fixedPoints);	//fill in the new mask
}


/*
	waits to die
*/
int main(int argc,char **argv){
	ROS_INFO("mapper starting");
	ros::init(argc,argv,"mapper");//name of this node

	tfl = new tf::TransformListener();
	ros::NodeHandle n;

	while (ros::ok()&&!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce();
	ROS_INFO("mapper done waiting");
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

	while(ros::ok())	ros::spin();

	ROS_INFO("Mapper terminated");
	return 0;
}
