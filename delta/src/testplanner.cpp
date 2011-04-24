#include <opencv/cv.h>
#include <opencv2/core/mat.hpp>
#include <iostream>
#include <opencv/highgui.h>
#include "plannerFuncs.cpp"

//uses some namespaces
using namespace cv;
using namespace std;

vector<Point2i> path;

// Subscribe, visualize, etc
int main(int argc,char **argv)
{
	/*
    // read in a picture
	cvNamedWindow("original");
	Mat mapImg = imread("/home/beth/Documents/Mobile_Robotics/delta/frame.jpg", 1);
	imshow("original", mapImg);
    
    // call aStar
	Point2i startPt, endPt; // yeah these need initialized
	startPt.x = startPt.y = 0;
	endPt.x = mapImg.size().width;
	endPt.y = mapImg.size().height;
	path = aStar(mapImg, startPt, endPt);
    */
    
    // lol jk fake points
    path.resize(10);
    for (int i=0; i<path.size(); i++)
    {
    	Point2i p;
    	p.x = i*50;
    	p.y = 250+10*i*pow(-1,i);
    	path[i] = p;
    }
    
    // pictures here
	//Mat img = Mat::zeros(mapImg.size().width, mapImg.size().height); // actual size!
	Mat_<Vec3f> img = Mat::zeros(500, 500, CV_32F); // for fake points
	cvNamedWindow("path");
	
	// put points into the matrix
	/*
	for (int i=0; i<path.size(); i++) 
	{
		Point2i p = path[i];
		circle(img, p, 1, Scalar(255,0,0), -1);
	}
	*/
	
	// ... or lines, whatevs
	// gradient doesn't akshully do much right now
	int initColor, curColor = 10;
	//circle(img, path[0], 10, Scalar(curColor, 0, 0), -1);
	for (int i=0; i<path.size()-1; i++)
	{
		line(img, path[i], path[i+1], Scalar(curColor, 0, 0));
		curColor = initColor + (255-initColor) * (i+1) / path.size();
		//cout << "New color is: " << curColor << "\n";
	}
	//circle(img, path[path.size()-1], 10, Scalar(curColor, 0, 0), -1);
	
	imshow("path", img);
	waitKey(-1);
}

