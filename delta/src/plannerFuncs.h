
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

#ifndef PLANNER_FUNCS
#define PLANNER_FUNCS

vector<Point2i> aStar (Mat_<char>& map, Point2i start, Point2i end);
vector<Point2f> convertToMap(vector<Point2i> victor, Point2f origin, double resolution);
vector<Point2f> reducePoints(vector<Point2f> victor);

class Node
{
	public:
	int x, y;
	Node* parent;
	double heuristic;
	double pathCost;
	
	Node(int x, int y, Node* parent, double heuristic, double pathCost)
	{
		this->x = x;
		this->y = y;
		this->parent = parent;
		this->heuristic = heuristic;
		this->pathCost = pathCost;
	}
	
	Node(const Node& N)
	{
		this->x = N.x;
		this->y = N.y;
		this->parent = N.parent;
		this->heuristic = N.heuristic;
		this->pathCost = N.pathCost;
	}

	Node()
	{
		x = y = -1;
		parent = NULL;
		heuristic = DBL_MAX;
		pathCost = DBL_MAX;
	}
	
};
#endif
