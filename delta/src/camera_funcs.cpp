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
#include "camera_funcs.h"

void getOrangeLines(Mat& img, vector<Vec4i>& lines)
{
  Mat src = img.clone();

  // Apply a gaussian filter
  Size ksize(7,7);
  GaussianBlur(img,src,ksize,2,2);

  Mat cdst, temp; 
  // I'm operating in hue space on the non-normalized image
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
      // Point to the right pixels in the source and destination mats
      srcpixel = src.at<Vec3b>(i,j);
      dstpixel = &dst.at<uchar>(i,j);

      // This pixel is redish-orange if R >= G >= B
      if( srcpixel[2] >= srcpixel[1] && srcpixel[1] >= srcpixel[0] )
      {
        // Hue = 60(G-B)/(R-B)
        val = 60.0*((float)srcpixel[1]-(float)srcpixel[0])/((float)srcpixel[2]-(float)srcpixel[0]);

        // Threshold based on hue
        if( val>15.0 && val<28.0 )
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

  // Run the line finding algorithm, and draw them on the source image
  HoughLinesP(dst, lines, 1, CV_PI/180, 60, 25, 10 );

  // Output how many lines you found
  ROS_INFO("Found %d lines",lines.size());

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

//these should be the closest point in the image to the robot.  I am just guessing that the image is 640 by 480 and that the robot is at the bottom of the image, in the middle
void GetNearest(&list<Point2i> Points, &list<Point2i> OrderedPoints, Point2i Target)
{
	double StartDist = 10000;
	double Dist = 10000;
	for (list<Point2i>::iterator it=Points.begin(); it!=Points.end(); i++)
	{
		//distance from start to origin
		TargetDist =  sqrt((Start->x-Target.x)*(Start->x-Target.x)+(Start->y-Target.y)*(Start->y-Target.y))
		Dist = sqrt((it->x-Target.x)*(it->x-Target.x)+(it->y-Target.x)*(it->y-Target.Y))
		if (Dist<TargetDist) Start = it;
	}

	NastyPolyLine.push_back(*Start);
	TempList.erase(Start);
}

#define IMAGE_ORIGIN_X 320
#define IMAGE_ORIGIN_Y 479

list<Point2i> linesToNastyPolyLine(vector<Vec4i> lines)
{
	list<Point2i> NastyPolyLine ();
	list<Point2i> TempList();

	for(i = 0; i<size(lines); i++)
	{
		TempList.push_back(Point2i(lines[i].x1, lines[i].y1));
		TempList.push_back(Point2i(lines[i].x2, lines[i].y2));
	}

	//find nearest to origin
	GetNearest(TempList, NastyPolyLine, Point2i(IMAGE_ORIGIN_X, IMAGE_ORIGIN_Y));

	while(TempList.size()>0)
	{
		GetNearest(TempList, NastyPolyLine, NastyPolyLine.end());
	}
	
	return NastyPolyLine;
}


list<Point2i> cleanNastyPolyLine(list<Point2i> NastyPolyLine, int NumRemaining)
{
	list<Point2i>::iterator leastSignificant;
	double minSignif; //short for minimum significance
	while(NastyPolyLine.size()>NumRemaining)
	{
		minSignif = 10000000; 
		for (list<Point2i>::iterator it=NastyPolyLine.begin(); it!=NastyPolyLine.end(); i++)
		{
			Point2i A = (*(it-1))-(*it);
			Point2i B = (*(it+1))-(*it);
			double angle = asin(A*B/(A.norm()*B.norm));
			double curSignif = fabs(180-angle)*A*B/(A.norm()+B.norm());
			if(curSignif<minSignif)
			{
				minSignif = curSignif;
				leastSignificant = it;
			}
		}
		NastyPolyline.erase(it);
	}	
}