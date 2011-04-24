
#include "command_publisher.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <list>
#include <vector>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <eecs376_msgs/PathSegment.h>
#include <eecs376_msgs/PathList.h>
#include <eecs376_msgs/CrawlerDesiredState.h>
#include <visualization_msgs/Marker.h>
#include "CSpaceFuncs.h"

using namespace cv;
using namespace std;
using namespace eecs376_msgs;

vector<Point2i> aStar (Mat map, Point2i start, Point2i end)
{
	//expand start
	
	//add all nearby start to the priority queue
	
	//while we have not found the path, 
	
		//expand the best thing in the priority queue
	
		//add it's neighbors to the priority queue
	
	//reconstruct the path
	//while the point we are reconstructing with is not the start point
	
		//add the current point to the list of points
	
		//current = parent
	
	//make list into vector and return it.
	
}

vector<Point2f> convertToMap(vector<Point2i> victor)
{
	//just iterate and convert in like 3 lines
}

vector<point2f> reducePoints(vector<Point2f> victor)
{
	//use an existing function here
}