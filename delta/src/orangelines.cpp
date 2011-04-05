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
using namespace cv;
using namespace std;

// Turns each line in "lines" into a series of points, spaced by pixelres
void linesToPoints(vector<Vec4i>& lines, vector<Point2i>& points, int pixelres)
{
  int npts = 0;
  Point2f p1f, p2f, pf;
  Point2i p;
  float xhat, yhat, length, traveledlength;

  // Convert the lines to points
  for( int i=0; i<lines.size(); i++ )
  {
    // Get the start, endpoint of the current line
    p1f.x = (float)lines[i][0];
    p1f.y = (float)lines[i][1];
    p2f.x = (float)lines[i][2];
    p2f.y = (float)lines[i][3];

    // Get unit vector from start to end
    length = sqrt((p2f.x-p1f.x)*(p2f.x-p1f.x) + (p2f.y-p1f.y)*(p2f.y-p1f.y));
    xhat = -(p1f.x-p2f.x)/length;
    yhat = -(p1f.y-p2f.y)/length;

    // Walk along the line toward p2 with resolution pixelres
    for( float traveledlength=0; traveledlength < length ; traveledlength+=(float)pixelres )
    {
      // Calculate the current point
      pf.x = p1f.x + xhat*traveledlength;
      pf.y = p1f.y + yhat*traveledlength;

      p = Point2i((int)pf.x, (int)pf.y);

      // Add current point to point list
      points.push_back(p);
      npts++;
    } 
  }
}


// Returns index of the nearest point to p, not further away than maxdist.
// Returns -1 if no such point is found
int getNearestNeighbor(vector<Point2i>& points, Point2i p, double max_detect_dist)
{
  int minindex = -1;
  float mindist, currdist;
  Point2i p1;

  mindist = 99999.9;

  for( int index=0; index<points.size(); index++ )
  {
    p1 = points[index];

    currdist = sqrt( (float)((p.x-p1.x)*(p.x-p1.x)) + (float)((p.y-p1.y)*(p.y-p1.y)) );
    if( currdist < mindist  && currdist < max_detect_dist )
    {
      mindist = currdist;
      minindex = index;
    }
  }

  return(minindex);
}

// Removes points near to p
void removeNearPoints(Point2i p, vector<Point2i>& points, double dist)
{
  float currdist;
  Point2i p1;
  for( int index=0; index<points.size(); index++ )
  {
    p1 = points[index];

    currdist = sqrt( (float)((p.x-p1.x)*(p.x-p1.x)) + (float)((p.y-p1.y)*(p.y-p1.y)) );
    if( currdist < dist )
    {
      index--;
      // remove it from further consideration
      points.erase(points.begin() + index);
    }
  }
}


void createPathFromPoints(Point2i start, vector<Point2i>& points, vector<Point2i>& pathpoints, int maxdist)
{
  // Add the closest point
  pathpoints.push_back(start);

  Point2i currpoint = Point2i(start);
  int nearest_neighbor_index = getNearestNeighbor(points, currpoint, 5000);

  // While a nearest neighbor exists
  while( nearest_neighbor_index != -1 )
  {
    // Get the closest point
    currpoint = points[nearest_neighbor_index];
    pathpoints.push_back(currpoint);

    // remove it from further consideration
    points.erase(points.begin() + nearest_neighbor_index);

    // Also remove near points from consideration
    removeNearPoints(currpoint,points,maxdist*.7);

    // Find the new nearest neighbor
    nearest_neighbor_index = getNearestNeighbor(points, currpoint, maxdist);
  }

  // Smooth out the path
  approxPolyDP(Mat(pathpoints),pathpoints,10,false);

}

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

        // Threshold based on hue
        if( val>15.0 && val<35.0 )
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
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "CameraNode");
	cout << "Camera Initialized" << endl;
	cout << "READY"  << endl;
	ROS_INFO("Camera Node Started");
 
<<<<<<< HEAD:delta/src/wesOrangeLines.cpp
  cvNamedWindow("detected lines");
  cvNamedWindow("original");
  Mat img = imread("/home/wes/Desktop/Mobile_Robotics/delta/frame.jpg", 1);
  imshow("original",img);
  Mat img2 = Mat(img);
=======
  Mat img = imread("/home/bk/code/dev_stacks/Mobile_Robotics/delta/frame.jpg", 1);
  
  // Get lines from the image
>>>>>>> c4ca925ba682aeefaa88000c9fd7287b8ebef71b:delta/src/orangelines.cpp
  vector<Vec4i> lines;
cout<<"begin orangeLines\n";
  getOrangeLines(img,lines);
<<<<<<< HEAD:delta/src/wesOrangeLines.cpp
cout<<"end orangeLines\n";
=======
  ROS_INFO("Found %d lines",lines.size());

  // Render the lines
  Mat temp = Mat(img);
>>>>>>> c4ca925ba682aeefaa88000c9fd7287b8ebef71b:delta/src/orangelines.cpp
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( temp, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
  }
<<<<<<< HEAD:delta/src/wesOrangeLines.cpp
cout<<"I drawed good stuffs\n";
  cvNamedWindow("cleanLine");
  cout<<"linesToNastyPolyLine\n";
  list<Point2i> PL = linesToNastyPolyLine(lines);
  cout<<"can haz clean path\n";
  PL = cleanNastyPolyLine(PL,5);
  cout<<"drawing more stuff\n";
  
  for(list<Point2i>::iterator it = PL.begin(); it!=PL.end(); it++)
  {
	  line(img2, *it, *it, Scalar(0,255,0),3, CV_AA);
  }
  cout<<"show stuff\n";
  imshow("cleanLine", img);
  imshow("detected lines",img);
  waitKey(-1);
  cvDestroyWindow("detected lines");
  cvDestroyWindow("original");
  cvDestroyWindow("thresholded image");
=======
  cvNamedWindow("detected lines");
  imshow("detected lines",temp);

  // Convert lines into points
  vector<Point2i> points;
  linesToPoints(lines, points, 3);

  // Render the points
  Mat temp2 = Mat(img);
  for( int i = 0; i < points.size()-1; i++ )
  {
    line( temp2, points[i], Point2i(points[i].x+1,points[i].y+1), Scalar(255,0,0), 1, CV_AA);
  }
  cvNamedWindow("points");
  imshow("points",temp2);

  // Make a path from the points
  Point2i start(1,479);
  vector<Point2i> pathpoints;
  createPathFromPoints(start, points, pathpoints,40);
  ROS_INFO("Got %d path points",pathpoints.size());
  
  // Render the path points
  Mat temp3 = Mat(img);
  for( int i = 0; i < pathpoints.size()-1; i++ )
  {
    line( temp3, pathpoints[i], pathpoints[i+1], Scalar(255,255,0), 1, CV_AA);
  }
  cvNamedWindow("new lines");
  imshow("new lines",temp3);
>>>>>>> c4ca925ba682aeefaa88000c9fd7287b8ebef71b:delta/src/orangelines.cpp

  waitKey(-1);
	while(ros::ok()){ros::spinOnce();}
	return 0;
}
