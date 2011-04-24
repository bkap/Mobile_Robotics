
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
#include <queue>
#include "CSpaceFuncs.h"
#include "plannerFuncs.h"
using namespace cv;
using namespace std;
using namespace eecs376_msgs;

vector<Point2i> aStar (Mat map, Point2i start, Point2i end)
{
	Node*** nodeList;
	priority_queue<Node> Q;
	
	nodeList = calloc(map.rows, sizeof(Node**));
	for(int i = 0; i<map.rows; i++)
	{
		nodeList[i] = calloc(map.cols, sizeof(Node*));
	}
	
	map = convertTo(map, CV_16U, 1, 129)
	
	//expand start
	
	nodeList[start.x][start.y] = new Node(start.x, start.y, NULL, heuristic(x,y,end), 0);
	
	//add all nearby start to the priority queue
	vector<Node> neighbors = getNeighbors(*nodeList[start.x][start.y], map, nodeList);
	for(int i = 0; i<neighbors.size(); i++)
	{
		Q.push(neighbors[i]);
	}
	
	//while we have not found the path, 
	bool done = false;
	Node current;
	while (!done)
	{
		//expand the best thing in the priority queue
		current = Q.top();
		Q.pop();
		nodeList[current.x][current.y] =  new Node(current);
		
		if(current.x = end.x&&current.y = end.y)
		{
			done = true;
			break;
		}
		
		//add it's neighbors to the priority queue
		vector<Node> neighbors = getNeighbors(current, map, nodeList);
		for(int i = 0; i<neighbors.size(); i++)
		{
			Q.push(neighbors[i]);
		}
	}
	
	list<Point2i> pathList;
	//reconstruct the path
	//while the point we are reconstructing which is not the start point
	while (current.parent!=NULL)
	{
		//add the current point to the list of points
		pathList.push_front(Point2i(current.x, current.y));
		
		//current = parent
		current = current.parent;
	}
	
	//make list into vector and return it.
	vector<Point2i> pathVec (pathList);
	return pathVec;
}

vector<Point2f> convertToMap(vector<Point2i> victor)
{
	//just iterate and convert in like 3 lines
}

vector<point2f> reducePoints(vector<Point2f> victor)
{
	//use an existing function here
}