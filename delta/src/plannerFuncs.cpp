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

//a heuristic function based on euclidean distance.  much less important than the path cost because of how path cost is defined
inline double heuristic(int x, int y, Point2i goal) {
	Point2i nodePoint(x, y);
	return norm(nodePoint - goal);
}

//computes the cost of this node, given the previous node's (expanding's) cost, and a value between 0 and 255 which roughly represents the likleyhood of a collision.  Due to how this is defined, the saftey of a path is weighted as being much more important than the shortness of the path.
inline double cost(Node expanding, double locCost) {
	return expanding.pathCost+locCost;
}


//this operator is defined in a strange way because the default c++ implementation requires a less than operator to be defined, but is a max-heap.  We just defined < as > which turned it into a min heap, which is required for A* search to function correctly.
bool operator<(Node a, Node b) 
{
	return a.heuristic+a.pathCost > b.heuristic + b.pathCost;
}

#define WALL_THRESHOLD 130 //anything greater than or equal to this number is a wall and planner will not attempt to pass through it.
//can result in crashes if no path exists

//gets the nearest neighbors in a vector of nodes, takes a node previous which we are finding the neighbors of, the current map so it can find path costs, the current nodeList so it can check for membership in the expanded set, and a goal so it can find a value for the heuristic function.
vector<Node> getNeighbors(Node previous, Mat_<int> &map, Node*** nodeList, Point2i goal) {
	vector<Node> nodes;

	Point2i directions[4] = {Point2i(1,0),Point2i(-1,0),Point2i(0,1), Point2i(0,-1)};
	for(int i = 0; i < 4; i++) {
		int newX = previous.x + directions[i].x;
		int newY = previous.y + directions[i].y;
		
		if(newX >= 0 && newX < map.rows && newY >= 0 && newY < map.cols) {
			//okay, we're on the map. Now let's check to see if we're allowed
			//to move here, based on the set of expanded nodes and the wall threhold
			
			if(nodeList[newX][newY] == NULL && map(newX,newY) < WALL_THRESHOLD) {
				//yay, we are allowed, make a new node
				Node n;
				n.pathCost = cost(previous,map(newX,newY));
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

//the actual aStar search itself, takes the CSpace map, a start point and an end point, and attempts to find a path between them
vector<Point2i> aStar (Mat_<char> &map_, Point2i start, Point2i end)
{
	ROS_INFO("astar: map size %dx%d",map_.rows,map_.cols);
	cout<<"start x,y "<<start.x<<","<<start.y<<"\n";
	cout<<"end x,y "<<end.x<<","<<end.y<<"\n";
	Node*** nodeList; //a triple pointer gives us a 2D array of pointers so we can just check to see if a thing exists in it by seeing if nodeList[x][y]!=NULL
	priority_queue<Node> Q;//the priority queue that C++ gives us, note that it is a min heap only due to operator overloading
	cout<<"a*1\n";

	// we need to convert from the format that the mapper hands us to a format that has 0 as the min not CHAR_MIN because A* doesn't work well if there exist negative path lengths
	Mat_<int> map = Mat::zeros(map_.rows, map_.cols, CV_32S);
	
	for(int i = 0; i<map.cols; i++)
	{
		for(int j = 0; j<map.rows; j++)
		{
			map(j,i) = ((int) map_(j,i))+129;
		}
	}
	
	//initialize everything to NULL indicating that there are no nodes in the table
	nodeList = (Node***) calloc(map.rows, sizeof(Node**)); 
	cout<<"map rows, cols "<<map.rows<<","<<map.cols<<"\n";
	for(int i = 0; i<map.rows; i++)
	{
		nodeList[i] = (Node**) calloc(map.cols, sizeof(Node*));
	}
	cout<<"a*2\n";
	//allocate temp space because convertTo gets mad if I don't
	//Mat temp;
	//map.copyTo(temp);
	cout<<"a*3\n";
	//temp.convertTo(map, CV_32S, 1, 129);

	//I don't know why we need this transpose, but we seem to have transposed things along the way, and this fixes the issue.
	map = map.t();
	//expand start
	cout<<"a*4\n";

	//do a start node, and expand it
	double h = heuristic(start.x,start.y,end);
	cout<<"heuristic of "<<h<<"\n";
	nodeList[start.x][start.y] = new Node(start.x, start.y, NULL, h, 0);
	//add all nearby start to the priority queue
	cout<<"a*4a\n";

	//get the neighbors of this start node and add them to the priority queue
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
		
		if(Q.size()==0) //see if we are about to die...
		{
			//give an error message if there is no path
			//the node may die at this point, then again, it might not... who knows
			//we assume that a valid path exists due to how the professor phrased the problem, so this should never be an issue, in theory, maybe, yeah...
			cout<<"AAAWWWW SHIIIITTTT!!!! THERE IS NO VALID PATH WTF WTF WTF!!!!!!!!!!!!!\n";
			break;
		}
		else Q.pop();//remove it because we don't need it anymore
		
		if(nodeList[current->x][current->y]!=NULL) continue;
		nodeList[current->x][current->y] =  new Node(*current);
		
		//check to see if we are at the end
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
	
	//free stuff so we don't memory leak
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

	//map2 was sometimes used to visualize stuff
	Mat_<int> map2 = Mat::zeros(map.rows, map.cols, CV_32S);
	
	for(int i = 0; i<map.cols; i++)
	{
		for(int j = 0; j<map.rows; j++)
		{
			map2(j,i) = ((int) (map(j,i)))*255;
			
		//	if(j%100==0)cout<<i<<","<<j<<","<<map(j,i)<<"\n";
		}
	}


//MORE DEFUNCT CODE IN A BIG BLOCK
	/*		vector<Point2f> path2;
		for(int z = 0; z<(int)pathVec.size(); z++)
		{
			path2.push_back(Point2f(pathVec[z].x, pathVec[z].y));
		}
	
		approxPolyDP(Mat(path2), path2, 2, false);
	//inRange(map,WALL_THRESHOLD,255,map2);
	cvNamedWindow("banned",CV_WINDOW_AUTOSIZE);
	for(int i = 1;i<path2.size();i++){
		line(map2,Point(path2[i-1].y,path2[i-1].x),Point(path2[i].y,path2[i].x),255*255);
	}
	imshow("banned",map2);
	waitKey(2);
	//cvNamedWindow("PATH",CV_WINDOW_AUTOSIZE);
	//imshow("PATH",map_);
	//waitKey(2);
*/

//return the vector 
	//delete current;
	return pathVec;
}

//converts a vector named victor to map coordinates from CSpace coordinates.  I don't know if this is used or not... it seems like duplicates exist.
vector<Point2f> convertToMap(vector<Point2i> victor, Point2f origin, double resolution)
{
	vector<Point2f> RetVal;
	for(int i = 0; i<(int)victor.size(); i++)
	{
		RetVal.push_back(Point2f(victor[i].x*resolution, victor[i].y*resolution)+origin);
	}
	return RetVal;
}

// the pathnode may have been used at some point in time for a smoothing algorithm that we found in a medical imaging journal. Probably not used
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

//this was the supposed path cleaning algorithm... wasn't used.
// see: http://www.google.com/url?sa=t&source=web&cd=7&ved=0CEAQFjAG&url=http%3A%2F%2Fciteseerx.ist.psu.edu%2Fviewdoc%2Fdownload%3Fdoi%3D10.1.1.1.8427%26rep%3Drep1%26type%3Dpdf&rct=j&q=remove%20insignificant%20line%20segments&ei=COd7TbTqDqrC0QGFwp30Aw&usg=AFQjCNFN6JN0kmN5l74DigQxRgSx9ApiEQ

//this code was used in the line following demo, but isn't currently used.  it did indeed work then... but yeah, ugly code
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

		//HOORAY FOR ITERATORS.  THEY DON'T HAVE A NEXT OR PREVIOUS POINTER WTF WTF WTF!!!!!!!
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

