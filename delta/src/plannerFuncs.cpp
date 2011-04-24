
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

inline double heuristic(int x, int y, Point2i goal) {
	Point2i nodePoint(x, y);
	return norm(nodePoint - goal);
}

inline double cost(Node expanding, double locCost) {
	return expanding.pathCost + locCost;
}

vector<Node> getNeighbors(Node previous, Mat map, Node*** nodeList, Point2i goal) {
	vector<Node> nodes;
	Point2i directions[4] = {Point2i(1,0),Point2i(-1,0),Point2i(0,1), Point2i(0,-1)};
	for(int i = 0; i < 4; i++) {
		int newX = previous.x + directions[i].x;
		int newY = previous.y + directions[i].y;
		if(newX > 0 && newX < map.rows && newY > 0 && newY < map.cols) {
			//okay, we're on the map. Now let's check to see if we're allowed
			//to move here
			if(nodeList[newX][newY] == NULL && map[newX][newY] != 256) {
				Node n;
				n.pathCost = cost(previous,map[newX][newY]);
				n.heuristic = heuristic(newX, newY, goal);
				n.x = newX;
				n.y = newY;
				n.Parent = &previous;
				nodes.push_back(n);
			}
		}
	}
	return nodes;
}

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

vector<Point2f> convertToMap(vector<Point2i> victor, Point2f origin, double resolution)
{
	//just iterate and convert in like 3 lines
}

vector<point2f> reducePoints(vector<Point2f> victor)
{
	//use an existing function here
}
