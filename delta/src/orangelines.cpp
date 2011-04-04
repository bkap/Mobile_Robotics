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
  ROS_INFO("Found %d lines",lines.size());
  for( int i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "CameraNode");
	cout << "Camera Initialized" << endl;
	cout << "READY"  << endl;
	ROS_INFO("Camera Node Started");
 
  cvNamedWindow("detected lines");
  cvNamedWindow("original");
  Mat img = imread("/home/bk/code/dev_stacks/Mobile_Robotics/delta/frame.jpg", 1);
  imshow("original",img);

  vector<Vec4i> lines;
  getOrangeLines(img,lines);
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }

  imshow("detected lines",img);
  waitKey(-1);
  cvDestroyWindow("detected lines");
  cvDestroyWindow("original");
  cvDestroyWindow("thresholded image");

	while(ros::ok()){ros::spinOnce();}
	return 0;
}
