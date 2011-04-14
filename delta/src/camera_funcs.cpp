//#include <camera_info_manager/camera_info_manager.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <eecs376_vision/lib_demo.h>
#include<tf/transform_listener.h> 
#include <stdio.h>
#include <list>
#include <algorithm>
#include "camera_funcs.h"
#include <sensor_msgs/Image.h>

int rlow = 180;
int rhigh = 255;
int glow = 0;
int ghigh = 120;
int blow = 0;
int bhigh = 60;


using namespace cv;
using namespace std;

//ros::NodeHandle N; 
sensor_msgs::CvBridge Bridge;
bool FirstRun = true;
//image_transport::ImageTransport I;
//image_transport::Publisher P;
Mat getOrangeLines(Mat& img, vector<Vec4i>& lines)
{
  
if(FirstRun)
{
FirstRun = false;
//I = N;
//P = I.advertise("IMAGE",1);
}
 Mat src = img.clone();
 //GaussianBlur(img,src, Size(15,15), 1.0,1.0);
  Mat cdst, temp; 
  // I'm operating on the non-normalized image
  //normalizeColor(src);

  // get a mat of the same dimensions as src, but floating point.
  Mat dst(src.rows, src.cols, CV_8U);

  Vec3b srcpixel;
  uchar* dstpixel;
  float val;

  // Access each pixel (remember they are ordered BGR)
  for( int i=0; i<src.rows; i++ )
  {
    for( int j=0; j<src.cols; j++ )
    {
      srcpixel = src.at<Vec3b>(i,j);
      dstpixel = &dst.at<uchar>(i,j);

      // This pixel is redish-orange if R >= G >= B
      if(true || srcpixel[2] >= srcpixel[1] && srcpixel[1] >= srcpixel[0] )
      {
        // Hue = 60(G-B)/(R-B)
      //  val = 60.0*((float)srcpixel[1]-(float)srcpixel[0])/((float)srcpixel[2]-(float)srcpixel[0]);
	//float brightness = (srcpixel[2] + srcpixel[1] + srcpixel[0]) /(3.0f * 255);
        // Threshold based on hue
        if( srcpixel[2] >= rlow && srcpixel[2] <= rhigh && 
		srcpixel[1] >= glow && srcpixel[1] <= ghigh &&
		srcpixel[0] >= blow && srcpixel[0] <= bhigh )
        {
          *dstpixel = 255; // The hue is redidsh-orange
        } else {
	  *dstpixel = 0;
	}
      }
      else // Does not meet criteria for redish-orange
      {
        *dstpixel = 0; // The hue is not reddish-orange.
      }
    }
  }

  // Erode and dilate to get rid of stray pixels
  //erode(dst, dst, Mat());
  dilate(dst, dst, Mat());
  dilate(dst, dst, Mat());
  cvtColor(dst, cdst, CV_GRAY2BGR);

  // Show the thresholded image
  //cvNamedWindow("thresholded image");
  //imshow("thresholded image",dst);

  HoughLinesP(dst, lines, 1, CV_PI/180, 40, 25, 10 );

  for( int i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }
  //IplImage temp13373r = src;
  //I.publish(Bridge.cvToImgMsg(temp13373r));

  // Show the thresholded image
  //cvNamedWindow("thresholded image");
  //imshow("thresholded image",dst);

  // Show the image with lines
  //cvNamedWindow("detected lines");
  //imshow("detected lines",src);
  return dst;
}

//asks the question is L or a point near L already in A.
bool IsIn(list<Point2f>::iterator A, list<Point2f> L, double minSeparation)
{
	for(list<Point2f>::iterator B = L.begin(); B!=L.end(); B++)
	{
		double d = sqrt((A->x-B->x)*(A->x-B->x)+(A->y-B->y)*(A->y-B->y));
		if(d<minSeparation)
		{
			return true;
		}
	} 
	return false;
}

//pops the nearest point to target that isn't within minSeparation
void popNearest(list<Point2f>& Points, list<Point2f>& OrderedPoints, Point2f Target, double minSeparation)
{
	bool found = false;
	list<Point2f>::iterator Nearest;
	double TargetDist = 10000;
	double Dist;
	//cout<<"\t\tGetNearest Init\n";
	for (list<Point2f>::iterator it=Points.begin(); it!=Points.end(); it++)
	{
		//distance from it to target
		Dist = sqrt((it->x-Target.x)*(it->x-Target.x)+(it->y-Target.y)*(it->y-Target.y));
		if (Dist<TargetDist&&!IsIn(it, OrderedPoints, minSeparation)) 
		{
			found = true;
			Nearest = it;
			TargetDist = Dist;
		}
	}
	//cout<<"\t\tfound some stuffs\n";
	if(found)OrderedPoints.push_back(*Nearest);
}

//*****WARNING THIS IS FOR TESTING PURPOSES ONLY**********
//*****IT DOESN'T TRANSFORM INTO ACTUAL COORDINATES****
list<Point2f> noTransform(list<Point2i> Pts)
{
	list<Point2f> result;
	for(list<Point2i>::iterator it = Pts.begin(); it!=Pts.end(); it++)
	{
		Point2f temp;
		temp.x = it->x;
		temp.y = it->y;
		result.push_back(temp);
	}
	return result;
}

list<Point2i> getUnsortedPoints(vector<Vec4i> lines)
{

	list<Point2i > TempList;
	cout<<"\tlTNPL:make TempList\n";
	for(int i = 0; i<lines.size(); i++)
	{
		TempList.push_back(Point2i(lines[i][0], lines[i][1]));
		TempList.push_back(Point2i(lines[i][2], lines[i][3]));
	}
	return TempList;
}


//minSeparation should be in meters for the robot code, but can be in pixels if noTransform is used instead of transformPts.
//the origin should be like 0,0 for the robot, but should be the bottom center of the image if noTransform is used.
list<Point2f> linesToNastyPolyLine(list<Point2f> Lines, double IMAGE_ORIGIN_X, double IMAGE_ORIGIN_Y, double minSeparation)
{
	list<Point2f > NastyPolyLine;
	cout << "POINTS" << endl;
	for(list<Point2f>::iterator it = Lines.begin(); it != Lines.end(); it++) {
		cout << (*it).x << "," << (*it).y << endl;
	}	
	cout << "YAY POINTS" << endl;
	//cout<<"\tlTNPL: getting first point\n";
	//find nearest to origin
	popNearest(Lines, NastyPolyLine, Point2i(IMAGE_ORIGIN_X, IMAGE_ORIGIN_Y), minSeparation);
	//cout<<"\tlTNPL: MOAR PTS!!!!\n";

	for(int i = 0; i< Lines.size(); i++)
	{
		popNearest(Lines, NastyPolyLine, *(--(NastyPolyLine.end())), minSeparation);
	}
	//cout<<"\tlTNPL:DONE!!!!\n";
	return NastyPolyLine;
}

//Cleans the PolyLine, may not be necessary after all.
list<Point2f> cleanNastyPolyLine(list<Point2f> NastyPolyLine, int NumRemaining)
{
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
