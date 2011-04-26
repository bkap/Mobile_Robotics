#include <opencv/cv.h>
#include <opencv2/core/mat.hpp>
#include <iostream>
#include <opencv/highgui.h>
#include "plannerFuncs.cpp"

//uses some namespaces
using namespace cv;
using namespace std;

vector<Point2i> path;

// Read in an image and convert it to a mat
// Black = min, white = max
Mat_<char> readImg()
{
	cvNamedWindow("original");
	Mat img = imread("/home/beth/Documents/Mobile_Robotics/delta/mapimg.jpg", 0); // read in grayscale img
	imshow("original", img);
	
	Mat_<char> converted = Mat::zeros(img.size().width, img.size().height, CV_8SC1);
	
	// loop through and convert
	// the values in the image mat will be between 0 and 255
	// want to normalize them to be between SCHAR_MIN and SCHAR_MAX
	// which coincidentally is also a range of 255, so just subtract SCHAR_MAX from it
	Mat_<char> testmat = Mat::zeros(100,100, CV_8SC1);
	for (int i=0; i<(int)testmat.size().width; i++)
		for (int j=0; j<(int)testmat.size().height; j++)
			testmat[i][j] = (char)((int) testmat[i][j]-SCHAR_MAX);
	
	return converted;
}

// Subscribe, visualize, etc
int main(int argc,char **argv)
{
	// Read in the image
    Mat_<char> mapImg = readImg();
    
    // call aStar
	Point2i startPt, endPt; // yeah these need initialized
	startPt.x = 0;
	startPt.y = 0;
	endPt.x = mapImg.size().width;
	endPt.y = mapImg.size().height;
	path = aStar(mapImg, startPt, endPt);
    
    // lol jk fake points
    /*
    path.resize(10);
    for (int i=0; i<(int)path.size(); i++)
    {
    	Point2i p;
    	p.x = i*50;
    	p.y = 250+10*i*pow(-1,i);
    	path[i] = p;
    }
    */
    
    // pictures here
	Mat img = Mat::zeros(mapImg.size().width, mapImg.size().height, CV_32F); // actual size!
	//Mat_<Vec3f> img = Mat::zeros(500, 500, CV_32F); // for fake points
	
	cvNamedWindow("path");
	
	// put in lines, gradient really isn't that noticeable/useful, so alternate green/blue
	int initColor = 10, curColor = 10;
	int initGreen = 255, curGreen = 255;
	//circle(img, path[0], 10, Scalar(curColor, curGreen, 0), -1);
	for (int i=0; i<(int)path.size()-1; i++)
	{
		(i%2==0) ? (curColor=0, curGreen=255) : (curColor=255, curGreen=0);
		line(img, path[i], path[i+1], Scalar(curColor, curGreen, 0));
		//curColor = initColor + (255-initColor) * (i+1) / path.size();
		//curGreen = initGreen - initGreen * (i+1) / path.size();
		cout << "New color is: " << curColor << ", " << curGreen << "\n";
	}
	//circle(img, path[path.size()-1], 10, Scalar(curColor, curGreen, 0), -1);
	imshow("path", img);
	
	waitKey(-1);
}

