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
	Mat img = imread("/home/wes/Desktop/Mobile_Robotics/delta/mapImg.bmp", 0); // read in grayscale img
	imshow("original", img);
		waitKey(-1);
	Mat_<char> converted = Mat::zeros(img.size().width, img.size().height, CV_8SC1);
	
	// loop through and convert
	// the values in the image mat will be between 0 and 255
	// want to normalize them to be between SCHAR_MIN and SCHAR_MAX
	// which coincidentally is also a range of 255, so just subtract SCHAR_MAX from it

	img.convertTo(converted, CV_8SC1, 1, -127);
	
	FileStorage F("someShit.xml", FileStorage::WRITE);
	F<<"someShit"<<img;
	
	

	//cvNamedWindow("converted");
	//imshow("converted", converted);
	//	waitKey(-1);
	
	return converted;
}

// Subscribe, visualize, etc
int main(int argc,char **argv)
{


	cout<<"1\n";

    

	// Read in the image
    Mat_<char> mapImg = readImg();

	Mat someShit;
   FileStorage F("someShit.xml", FileStorage::READ);
	F["someShit"]>>someShit;
	imshow("someShit", someShit);
		waitKey(-1);

    	cout<<"2\n";
    // call aStar
	Point2i startPt, endPt; // yeah these need initialized
	startPt.x = 0;
	startPt.y = 0;
	endPt.x = mapImg.size().width;
	endPt.y = mapImg.size().height;
	path = aStar(mapImg, startPt, endPt-Point2i(1,1));
	
	vector<Point2f> path2;
	for(int i = 0; i<(int)path.size(); i++)
	{
		path2.push_back(Point2f(path[i].x, path[i].y));
	}
	
	approxPolyDP(Mat(path2), path2, 2, false);

	path.clear();
	
	for(int i = 0; i<(int) path2.size(); i++)
	{
		path.push_back(Point2i(path2[i].x, path2[i].y));
	}
	
    	cout<<"3\n";
  
    	cout<<"4\n";

    

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
	    cout<<"5\n";

	// put in lines, gradient really isn't that noticeable/useful, so alternate green/blue
	int initColor = 10, curColor = 10;
	int initGreen = 255, curGreen = 255;
	//circle(img, path[0], 10, Scalar(curColor, curGreen, 0), -1);
	for (int i=0; i<(int)path.size()-1; i++)
	{
		(i%2==0) ? (curColor=0, curGreen=255) : (curColor=255, curGreen=0);
		line(mapImg, path[i], path[i+1], Scalar(curColor, curGreen, 0));
		//curColor = initColor + (255-initColor) * (i+1) / path.size();
		//curGreen = initGreen - initGreen * (i+1) / path.size();
		cout << "New color is: " << curColor << ", " << curGreen << "\n";
	}
cout<<"6\n";
	//circle(img, path[path.size()-1], 10, Scalar(curColor, curGreen, 0), -1);

	imshow("path", mapImg);
	
	waitKey(-1);
}

