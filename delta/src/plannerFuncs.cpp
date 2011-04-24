
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
#include <assert.h>
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
	vector<Node> neighbors = getNeighbors(*nodeList[start.x][start.y], map, nodeList, end);
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
		vector<Node> neighbors = getNeighbors(current, map, nodeList, end);
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

inline double heuristic(int x, int y, Point2i goal) {
	Point2i nodePoint(x, y);
	return norm(nodePoint - goal);
}

inline double cost(Node expanding, double locCost) {
	return expanding.pathCost + locCost;
}

vector<Point2f> convertToMap(vector<Point2i> victor, Point2f origin, double resolution)
{
	vector<Point2f> RetVal;
	for(int i = 0; i<victor.size(); i++)
	{
		RetVal.push_back(Point2f(victor[i].x*resolution, victor[i].y*resolution)+origin);
	}
	return RetVal;
}

class PathNode
{
	public:
	Point2f Pose;
	double Significance
	PathNode(Point2f Pose, double Significance);
}

bool operator<(PathNode A, PathNode B)
{
	return A.Significance<B.Significance;
}

vector<Point2f> cleanPath(vector<Point2f> NastyPolyLine, int NumRemaining)
{
	assert(1==0);//DON'T USE THIS I DIDN'T DO IT !!!!!!!!!!!!!!!! IT DOESN'T WORK AT  ALL!!!!!!!!!!!!!!
	priority_queue Q;
	list<Point2f>::iterator leastSignificant;
	double minSignif; //short for minimum significance
	int i =0;
	while(NastyPolyLine.size()>NumRemaining)
	{
		//cout<<"loopin "<<i++<<"\n";
		minSignif = 10000000; 
		for (list<Point2f>::iterator it=++(++(NastyPolyLine.begin())); it!=--(NastyPolyLine.end()); it++)
		{
			Point2f A = (*(--it))-(*(++it));
			Point2f B = (*(++it))-(*(--it));
			cout << A.x << "," << A.y << endl;
			double cosTheta = A.ddot(B)/(norm(A)*norm(B));
			cout << cosTheta << endl;
			double angle = acos(cosTheta);
			double curSignif = fabs(3.14159-angle)*A.ddot(B)/(norm(A)+norm(B));
			//if(cosTheta>0) curSignif = -100;
			cout << curSignif << endl;
			if(curSignif<minSignif)
			{
				cout << "new candidate" << endl;
				minSignif = curSignif;
				leastSignificant = it;
			}
		}
		cout << "deleting" << endl;
		NastyPolyLine.erase(leastSignificant);
		cout << "deleted" << endl;
	}	
	return NastyPolyLine;
}

