#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

#include <eecs376_vision/lib_demo.h>

#include <eecs376_vision/cvblob.h>

#include <time.h>

int rlow = 180;
int rhigh = 255;
int glow = 50;
int ghigh = 100;
int blow = 0;
int bhigh = 60;

using namespace cv;
using namespace cvb;

void normalizeColors(const cv::Mat& src, cv::Mat& out) {
  Mat temp;
  src.convertTo(temp, CV_32F); // convert to floats first?
  vector<Mat> mats; //make a vector of Mats to hold each channel
  split(temp, mats); //split the src image into B, G and R channels
  Mat total = mats[0] + mats[1] + mats[2]; //sum of B+G+R
  //Mat total = mats[0].mul(mats[0]) + mats[1].mul(mats[1]) + mats[2].mul(mats[2]); //sum of B+G+R
  //sqrt(total, total);
  mats[0] = mats[0] / total; // normalize B channel
  mats[1] = mats[1] / total; // normalize G channel
  mats[2] = mats[2] / total; // normalize R channel
  merge(mats, temp); // merge the individual channels back into a BGR image
  temp.convertTo(out, CV_8U, 255);
}

void findLines(const cv::Mat& src, cv::Mat& out) {
  Mat temp, color_temp; //setup some temps
  cvtColor(src, temp, CV_BGR2GRAY); //convert to grayscale for the edge detector
  //Sobel(temp, temp, CV_8U, 1, 1);
  Canny( temp, temp, 50, 200, 3 ); //run Canny edge detector with some default values
  cvtColor( temp, color_temp, CV_GRAY2BGR ); //Convert Canny edges back to 3-channel

  vector<Vec4i> lines;
  HoughLinesP( temp, lines, 1, CV_PI/180, 80, 30, 10 ); //Find lines in the Canny image
  for( size_t i = 0; i < lines.size(); i++ )
  {
    line( color_temp, Point(lines[i][0], lines[i][1]),
        Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 ); //Draw
  }
  out = color_temp;
}

CvPoint2D64f blobfind(const cv::Mat& src, cv::Mat& out)
{
  Mat temp;

  //cvtColor(src, temp, CV_BGR2HSV);
  temp = src;

  //Make a vector of Mats to hold the invidiual B,G,R channels
  vector<Mat> mats;

  //Split the input into 3 separate channels
  split(temp, mats);

  // Set all values below value to zero, leave rest the same
  // Then inverse binary threshold the remaining pixels
  // Threshold blue channel
  threshold(mats[0], mats[0], bhigh, 255, THRESH_TOZERO_INV);
  threshold(mats[0], mats[0], blow, 255, THRESH_BINARY);
  // Threshold green channel
  threshold(mats[1], mats[1], ghigh, 255, THRESH_TOZERO_INV);
  threshold(mats[1], mats[1], glow, 255, THRESH_BINARY);
  // Threshold red channel
  threshold(mats[2], mats[2], rhigh, 255, THRESH_TOZERO_INV);
  threshold(mats[2], mats[2], rlow, 255, THRESH_BINARY);

  multiply(mats[0], mats[1], out);
  multiply(out, mats[2], out);

  erode(out, out, Mat());

  dilate(out, out, Mat(), Point(-1,-1), 30);

  IplImage temp1 = out;
  IplImage temp2 = temp;
  IplImage *labelImg=cvCreateImage(cvGetSize(&temp1), IPL_DEPTH_LABEL, 1);
  CvBlobs blobs;
  unsigned int result=cvLabel(&temp1, labelImg, blobs);
  cvRenderBlobs(labelImg, blobs, &temp2, &temp2);

  CvLabel greatestBlob = cvGreaterBlob(blobs);
  CvPoint2D64f center;
  center.x = 0;//src.size().width/2;
  center.y = 0;//src.size().height/2;
  if (greatestBlob > 0) {
    center = cvCentroid(blobs[greatestBlob]);
  }

  out = temp;
  cvReleaseImage(&labelImg);
  return center;
}
