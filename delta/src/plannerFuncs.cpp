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

inline double heuristic(int x, int y, Point2i goal) {
	Point2i nodePoint(x, y);
	return norm(nodePoint - goal);
}

inline double cost(Node expanding, double locCost) {
	return expanding.pathCost+10*locCost;//+ exp(locCost/20.0);
}


bool operator<(Node a, Node b) 
{
	return a.heuristic+a.pathCost > b.heuristic + b.pathCost;
}

#define WALL_THRESHOLD 200

vector<Node> getNeighbors(Node previous, Mat_<char> &map, Node*** nodeList, Point2i goal) {
	vector<Node> nodes;

	Point2i directions[4] = {Point2i(1,0),Point2i(-1,0),Point2i(0,1), Point2i(0,-1)};
	for(int i = 0; i < 4; i++) {
		int newX = previous.x + directions[i].x;
		int newY = previous.y + directions[i].y;
		
		if(newX >= 0 && newX < map.rows && newY >= 0 && newY < map.cols) {
			//okay, we're on the map. Now let's check to see if we're allowed
			//to move here
		
			if(nodeList[newX][newY] == NULL && map.at<int>(newX,newY) < WALL_THRESHOLD) {
			
				Node n;
				n.pathCost = cost(previous,map.at<int>(newX,newY));
				n.heuristic = heuristic(newX, newY, goal);
				n.x = newX;
				n.y = newY;
				n.parent = nodeList[previous.x][previous.y];
				nodes.push_back(n);
			}
		}
	}
	return nodes;
}

vector<Point2i> aStar (Mat_<char> &map, Point2i start, Point2i end)
{
	cout<<"start x,y "<<start.x<<","<<start.y<<"\n";
	cout<<"end x,y "<<end.x<<","<<end.y<<"\n";
	Node*** nodeList;
	priority_queue<Node> Q;
	cout<<"a*1\n";
	nodeList = (Node***) calloc(map.rows, sizeof(Node**));
	cout<<"map rows, cols "<<map.rows<<","<<map.cols<<"\n";
	for(int i = 0; i<map.rows; i++)
	{
		nodeList[i] = (Node**) calloc(map.cols, sizeof(Node*));
	}
	cout<<"a*2\n";
	//allocate temp space because convertTo gets mad if I don't
	Mat temp;
	map.copyTo(temp);
	cout<<"a*3\n";
	temp.convertTo(map, CV_32S, 1, 129);
	map = map.t();
	//expand start
	cout<<"a*4\n";
	double h = heuristic(start.x,start.y,end);
	cout<<"heuristic of "<<h<<"\n";
	nodeList[start.x][start.y] = new Node(start.x, start.y, NULL, h, 0);
	//add all nearby start to the priority queue
	cout<<"a*4a\n";
	vector<Node> neighbors = getNeighbors(*(nodeList[start.x][start.y]), map, nodeList, end);
	cout<<"a*4b\n";
	for(int i = 0; i<(int)neighbors.size(); i++)
	{
		Q.push(neighbors[i]);
	}
	cout<<"a*5\n";
	//while we have not found the path, 
	bool done = false;
	Node *current = new Node();
	while (!done)
	{
		//expand the best thing in the priority queue
		*current = Q.top();
		
		if(Q.size()==0) 
		{
			cout<<"AAAWWWW SHIIIITTTT!!!! THERE IS NO VALID PATH WTF WTF WTF!!!!!!!!!!!!!\n";
			break;
		}
		else Q.pop();
		
		if(nodeList[current->x][current->y]!=NULL) continue;
		nodeList[current->x][current->y] =  new Node(*current);
		
		if(current->x == end.x&&current->y == end.y)
		{
			done = true;
			break;
		}
		
		//add it's neighbors to the priority queue
		vector<Node> neighbors = getNeighbors(*current, map, nodeList, end);
		for(int i = 0; i<(int)neighbors.size(); i++)
		{
			Q.push(neighbors[i]);
		}
	}
	cout<<"a*6\n";
	list<Point2i> pathList;
	//reconstruct the path
	//while the point we are reconstructing which is not the start point
	while (current->parent!=NULL)
	{
		//add the current point to the list of points
		pathList.push_front(Point2i(current->x, current->y));

		//current = parent
		current = current->parent;
	}
	cout<<"a*7\n";
	//make list into vector and return it.
	vector<Point2i> pathVec (pathList.begin(),pathList.end());
	
	
	for(int i = 0; i<map.rows; i++)
	{
		for(int j = 0; j<map.cols; j++)
		{
			if(nodeList[i][j]!=NULL)
			{
				delete nodeList[i][j];
			}
		}
		free(nodeList[i]);
	}
	
	free(nodeList);
	
	//delete current;
	return pathVec;
}

vector<Point2f> convertToMap(vector<Point2i> victor, Point2f origin, double resolution)
{
	vector<Point2f> RetVal;
	for(int i = 0; i<(int)victor.size(); i++)
	{
		RetVal.push_back(Point2f(victor[i].x*resolution, victor[i].y*resolution)+origin);
	}
	return RetVal;
}

class PathNode
{
	public:
	Point2f Pose;
	double Significance;
	PathNode(Point2f Pose, double Significance);
};

bool operator<(PathNode A, PathNode B)
{
	return A.Significance<B.Significance;
}

vector<Point2f> cleanPath(vector<Point2f> NastyPolyLineOrig, int NumRemaining)
{
	list<Point2f> NastyPolyLine(NastyPolyLineOrig.begin(), NastyPolyLineOrig.end());
	list<Point2f>::iterator leastSignificant;
	double minSignif; //short for minimum significance
	int i =0;
	while((int)NastyPolyLine.size()>NumRemaining)
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
				minSignif = curSignif;
				leastSignificant = it;
			}
		}
		
		NastyPolyLine.erase(leastSignificant);
		
	}	
	return vector<Point2f>(NastyPolyLine.begin(), NastyPolyLine.end());
}

