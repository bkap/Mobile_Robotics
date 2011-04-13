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

using namespace cv;
using namespace std;

void getOrangeLines(Mat& img, vector<Vec4i>& lines)
{
  Mat src = img.clone();

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
      if( srcpixel[2] >= srcpixel[1] && srcpixel[1] >= srcpixel[0] )
      {
        // Hue = 60(G-B)/(R-B)
        val = 60.0*((float)srcpixel[1]-(float)srcpixel[0])/((float)srcpixel[2]-(float)srcpixel[0]);
	float brightness = ((float)srcpixel[2] + (float)srcpixel[1] + (float)srcpixel[0]) / (3.0f * 255);
        // Threshold based on hue
        if( val>15.0 && val<35.0 && brightness > 0.7)
        {
          *dstpixel = 255; // The hue is redidsh-orange
        }
      }
      else // Does not meet criteria for redish-orange
      {
        *dstpixel = 0.0; // The hue is not reddish-orange.
      }
    }
  }

  // Erode and dilate to get rid of stray pixels
  erode(dst, dst, Mat());
  dilate(dst, dst, Mat());

  cvtColor(dst, cdst, CV_GRAY2BGR);

  // Show the thresholded image
  cvNamedWindow("thresholded image");
  imshow("thresholded image",dst);

  HoughLinesP(dst, lines, 1, CV_PI/180, 60, 25, 10 );

  for( int i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }

  // Show the thresholded image
  //cvNamedWindow("thresholded image");
  //imshow("thresholded image",dst);

  // Show the image with lines
  //cvNamedWindow("detected lines");
  //imshow("detected lines",src);
}

//asks the question is L or a point near L already in A.
bool IsIn(list<Point2d>::iterator A, list<Point2d> L, double minSeparation)
{
	for(list<Point2d>::iterator B = L.begin(); B!=L.end(); B++)
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
void popNearest(list<Point2d>& Points, list<Point2d>& OrderedPoints, Point2d Target, double minSeparation)
{
	bool found = false;
	list<Point2d>::iterator Nearest;
	double TargetDist = 10000;
	double Dist;
	//cout<<"\t\tGetNearest Init\n";
	for (list<Point2d>::iterator it=Points.begin(); it!=Points.end(); it++)
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
list<Point2d> noTransform(list<Point2i> Pts)
{
	list<Point2d> result;
	for(list<Point2i>::iterator it = Pts.begin(); it!=Pts.end(); it++)
	{
		Point2d temp;
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
list<Point2d> linesToNastyPolyLine(list<Point2d> Lines, double IMAGE_ORIGIN_X, double IMAGE_ORIGIN_Y, double minSeparation)
{
	list<Point2d > NastyPolyLine;
	
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
list<Point2d> cleanNastyPolyLine(list<Point2d> NastyPolyLine, int NumRemaining)
{
	list<Point2d>::iterator leastSignificant;
	double minSignif; //short for minimum significance
	int i =0;
	while(NastyPolyLine.size()>NumRemaining)
	{
		//cout<<"loopin "<<i++<<"\n";
		minSignif = 10000000; 
		for (list<Point2d>::iterator it=++(NastyPolyLine.begin()); it!=--(NastyPolyLine.end()); it++)
		{
			Point2i A = (*(--it))-(*(++it));
			Point2i B = (*(++it))-(*(--it));
			double cosTheta = A.ddot(B)/(norm(A)*norm(B));
			double angle = acos(cosTheta);
			double curSignif = fabs(180-angle)*A.ddot(B)/(norm(A)+norm(B));
			if(cosTheta>0) curSignif = -100;
			if(curSignif<minSignif)
			{
				minSignif = curSignif;
				leastSignificant = it;
			}
		}
		NastyPolyLine.erase(leastSignificant);
	}	
	return NastyPolyLine;
}
