#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/LaserScan.h>
#include <eecs376_vision/lib_demo.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include "cvFuncs.h"

using namespace cv;
using namespace std;


#define PRECAL 0

/*
	This should now work.  There is some code in the image callback which flips the image and is only necessary when testing from the harlie bagfile and should otherwise be disabled

	The code if(PRECAL){...} in the image callback converts the camera image to the birdseye view

	perspectiveTransform(Mat(CoordsInCameraIJ),CoordsInBaseXY,viewToBase) transforms points from the camera frame into base_link
		no homogeneous co-ordinates needed

*/

bool last_scan_valid; // This is set to true by the LIDAR callback if it detects a plausible location for the rod.
// Also declare a point datatype here to hold the location of the rod as detected by LIDAR
Point2f rodLocation; //set this the x,y coordinate of any rod found
vector<Point2f> baseXY; 	//aggregator for base_link co-ordinates
vector<Point2f> birdsIJ;	//aggregator for birds eye pixel co-ordinates
vector<Point2f> viewIJ;		//aggregator for view pixel co-ordinates

Mat_<double> viewToBaseInv;  	//transform between camera pixel co-ordinates and base_link (x,y)
Mat_<double> viewToOrthoInv;	//transform between camera and birds-eye pixel co-ordinates
Mat_<double> orthoToBaseInv;	//transform between birds-eye pixel co-ordinates and base_link (x,y)

Point2f orthoBounds(4,2);	//defines viewable area of overhead projection as: x in (0,orthoBOunds.x), y in (-orthoBounds.y,orthoBounds.y)
double ppm = 200;		//pixels/m in ortho image

Size orthoImageSize(2 * orthoBounds.y * ppm + 1 , orthoBounds.x * ppm + 1);

Mat lastImage;			//last image pulled from the callback
Mat firstImage;


template <typename T>
void readMat(cv::Mat_<T>& mat, char* file){
	ifstream* infile = new ifstream(file,ifstream::in&ifstream::binary);
	int rows = 0,cols = 0;
	(*infile)>>rows;
	(*infile)>>cols;
	mat = Mat_<T>(rows,cols);
	T val;
	for(int i=0;i<mat.rows;i++){
		for(int j=0;j<mat.cols;j++){
			(*infile)>>val;
			mat(i,j)=val;
		}
	}
	infile->close();
}
template <typename T>
void writeMat(cv::Mat_<T>& mat, char* file){
	ofstream* ofile = new ofstream(file,ofstream::out&ofstream::binary);
	(*ofile)<<mat.rows<<" ";
	(*ofile)<<mat.cols<<" ";
	
	for(int i = 0;i<mat.rows;i++){
		for(int j = 0;j<mat.cols;j++){
			(*ofile)<<(T) mat(i,j)<<" ";
		}
	}
	ofile->close();
}




class DemoNode {
  public:
    DemoNode();
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    ros::NodeHandle nh_; // Made this public to access it and subscribe to the LIDAR
  private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image_;
    ros::Subscriber sub_lidar_;
    image_transport::Publisher image_pub_;
};

DemoNode::DemoNode():
  it_(nh_)
{
  sub_image_ = it_.subscribe("image", 1, &DemoNode::imageCallback, this);
  sub_lidar_ = nh_.subscribe<sensor_msgs::LaserScan>("lidar",1,&DemoNode::lidarCallback,this);
  image_pub_ = it_.advertise("demo_image", 1);
}

int filterLIDAR(sensor_msgs::LaserScan& scan){
  int num_filtered = 0;
  int num_points = scan.ranges.size();
  for( int i=1; i<num_points-1; i++ )
  {
    // This pixel is a candidate for filtering iff it is at max range
    if( scan.ranges[i]>=scan.range_max-1.5 )
    {
      scan.ranges[i] = 0.5*(scan.ranges[i-1]+scan.ranges[i+1]);
      num_filtered++;
    }
  }
  // Special cases for first/last pixels
  if( (scan.ranges[0]<scan.range_max) && (scan.ranges[1]>=scan.range_max-1.5) )
  {
    scan.ranges[0] = scan.ranges[1];
    num_filtered++;
  }
  if( (scan.ranges[num_points-2]<scan.range_max) && (scan.ranges[num_points-1]>=scan.range_max-1.5) )
  {
    scan.ranges[num_points-1] = scan.ranges[num_points-2];
    num_filtered++;
  }
	return num_filtered;
}

bool findRod(sensor_msgs::LaserScan& scan){
float rodSepThresh  = 0.4; //min dist of rod from surrounding
float rodDispThresh = 0.05;//max radial diff between rod points
float rodWidthThresh= 3;   //max pings of a rod
float rodDistThresh = 4;   //max distance to rod
bool maybeRod = false;
int rodStart = -1;
float minr = 1000;
float maxr = 0;

last_scan_valid = false;

for(unsigned int i=1;i<scan.ranges.size()-1;i++){
        if(!maybeRod && (scan.ranges[i-1] - scan.ranges[i])>rodSepThresh){
            maybeRod = true;
            rodStart = i;
            minr = scan.ranges[i];
            maxr = scan.ranges[i];
        }
        if(maybeRod){
            minr = scan.ranges[i] < minr? scan.ranges[i]:minr;
            maxr = scan.ranges[i] > maxr? scan.ranges[i]:maxr;
 	    if(scan.ranges[i+1] - scan.ranges[i] > rodSepThresh){
	         if(i-rodStart >= rodWidthThresh || maxr - minr > rodDispThresh || maxr > rodDistThresh){
        	     maybeRod = false;
            	 }
	         else{
                    float r = scan.ranges[i];//(maxr + minr) / 2;
		    float t = CV_PI * ( ((float) (rodStart+i))/360.0 -0.5);
			cout<<"rod at: "<<r<<" m "<<t*180/CV_PI<<endl;
                    last_scan_valid = true;
                    rodLocation.x = r*cos(t);
                    rodLocation.y = r*sin(t);
                    maybeRod = false;
                    maxr = 0;
                    minr = 1000;
		    break;
            	}
	    }
	}
 }
return last_scan_valid;
}

// Callback triggered whenever you receive a laser scan
void DemoNode::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  sensor_msgs::LaserScan scan = *msg;

//  cout<<"got scan in frame -> "<<scan.header.frame_id<<"\n";

  // First, filter the laser scan to remove random bad pings.  Search through the laser scan, and pick out all points with max range.  Replace these points by the average of the two points on either side of the bad point.
  int num_filtered = filterLIDAR(scan);
  bool foundRod = findRod(scan);

	if(foundRod){
		ROS_INFO("LIDAR detected rod after smoothing %d bad points",num_filtered);
	}
}

void DemoNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//  cout<<"IMAGE CALLBACK\n";
  sensor_msgs::CvBridge bridge;
  cv::Mat image;
  cv::Mat output;
  static bool first = true;
  try
  {
    Mat im_ = cv::Mat(bridge.imgMsgToCv(msg, "bgr8"));
    flip(im_,image,-1);
    lastImage = image.clone();
	if(first)	firstImage = image.clone();
	first = false;
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'. E was %s", msg->encoding.c_str(), e.what());
  }
  try {
    normalizeColors(image, output);
    CvPoint2D64f Center = blobfind(image, output);
    cv::Point2f center (Center.x, Center.y);
    if(center.x>0 && last_scan_valid){
//	cout<<"Matched a blob and a ping.\n";
	viewIJ.push_back(Point2f(center.y, center.x));	//image points are in (i,j) instead of (x,y) so co-ordinates are reversed
	baseXY.push_back(Point2f(rodLocation.x,rodLocation.y));
	birdsIJ.push_back(Point2f(
					orthoImageSize.height - rodLocation.x * ppm,
					(orthoImageSize.width-1)/2 - rodLocation.y * ppm));
    }
	if(PRECAL){
		warpPerspective(image.t(),output,viewToOrthoInv,Size(orthoImageSize.height,orthoImageSize.width),WARP_INVERSE_MAP);
		output = output.t();
    }
    IplImage temp = output;
   image_pub_.publish(bridge.cvToImgMsg(&temp, "bgr8"));
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert to 'bgr8'. Ex was %s", e.what());
  }
}

int main(int argc, char **argv)
{
	if(PRECAL){
	        //readMat<double>(viewToBaseInv,"/home/connor/Code/Mobile_Robotics/delta/viewToBaseInv");
	        //readMat<double>(viewToOrthoInv,"/home/connor/Code/Mobile_Robotics/delta/viewToOrthoInv");
	        //readMat<double>(orthoToBaseInv,"/home/connor/Code/Mobile_Robotics/delta/orthoBaseInv");
	}
  ros::init(argc, argv, "eecs376_vision_demo1");
  DemoNode motion_tracker;
  cvNamedWindow("birdseye",CV_WINDOW_AUTOSIZE); //these cv* calls are need if you want to use cv::imshow anywhere in your program
  //cvStartWindowThread();
  ROS_INFO("Calibration procedure started");
  ros::Rate naptime(75);

  while(ros::ok() && (PRECAL || viewIJ.size()<100))
  {
        naptime.sleep();
  	ros::spinOnce();
  }
//	ros::spin();
	if(PRECAL){
		return 0;
	}
	cout<<"beginning calibration"<<endl;



	/*	Initialization	*/
	Mat viewIJ_ = Mat(viewIJ);
	Mat baseXY_= Mat(baseXY);
	Mat birdsIJ_= Mat(birdsIJ);

	vector<Point2f> orthoCornersBaseXY;	//corners of birds-eye view in base_link co-ordinates (x,y)
	orthoCornersBaseXY.push_back(Point2f(orthoBounds.x,orthoBounds.y));
	orthoCornersBaseXY.push_back(Point2f(orthoBounds.x,-orthoBounds.y));
	orthoCornersBaseXY.push_back(Point2f(0,orthoBounds.y));
	orthoCornersBaseXY.push_back(Point2f(0,-orthoBounds.y));


	vector<Point2f> viewCornersIJ;		//corners of camera image in image co-ordinates (i,j)
	viewCornersIJ.push_back(Point2f(0,0));
	viewCornersIJ.push_back(Point2f(0,lastImage.size().width-1));
	viewCornersIJ.push_back(Point2f(lastImage.size().height-1,0));
	viewCornersIJ.push_back(Point2f(lastImage.size().height-1,lastImage.size().width-1));

	vector<Point2f> orthoCornersIJ;		//corners of birds-eye view image in image co-ordinates (i,j)
	orthoCornersIJ.push_back(Point2f(0,0));
	orthoCornersIJ.push_back(Point2f(0,orthoImageSize.width-1));
	orthoCornersIJ.push_back(Point2f(orthoImageSize.height-1,0));
	orthoCornersIJ.push_back(Point2f(orthoImageSize.height-1,orthoImageSize.width-1));

	Mat_<Point2f> orthoCornersIJ_,viewCornersBaseXY_;
	vector<unsigned char> mask1,mask2,mask3;
	/*	Computation	*/
	Mat_<double> viewToBaseInv = findHomography(baseXY_,viewIJ_,mask1,RANSAC,.5);	//transform between camera pixel co-ordinates and base_link (x,y)
	Mat_<double> viewToOrthoInv= findHomography(birdsIJ_,viewIJ_,mask2,RANSAC,.5);	//transform between camera and birds-eye pixel co-ordinates
	Mat_<double> orthoToBaseInv= findHomography(baseXY_,birdsIJ_,mask3,RANSAC,.5);	//transform between birds-eye pixel co-ordinates and base_link (x,y)
	Mat_<double> orthoToBase,viewToBase,viewToOrtho;
	invert(orthoToBaseInv,orthoToBase);
	invert(viewToBaseInv,viewToBase); 
	invert(viewToOrthoInv,viewToOrtho);
	/*	Logging and Verification	*/
	writeMat<double>(viewToBaseInv,"/home/connor/Code/Mobile_Robotics/delta/viewToBaseInv");
	writeMat<double>(viewToOrthoInv,"/home/connor/Code/Mobile_Robotics/delta/viewToOrthoInv");
	writeMat<double>(orthoToBaseInv,"/home/connor/Code/Mobile_Robotics/delta/baseToOrthoInv");

	cout<<"viewToBase inliers "<<countNonZero(Mat(mask1))<<"\nviewToOrtho inliers "<<countNonZero(Mat(mask2))<<"\northoToBase inliers "<<countNonZero(Mat(mask3))<<endl;
	cout<<"Birds eye image size width,height: "<<orthoImageSize.width<<","<<orthoImageSize.height<<endl;

	perspectiveTransform(Mat(orthoCornersBaseXY),orthoCornersIJ_,orthoToBaseInv);
	cout<< "project base_link co-ordinates:\n"<<Mat(orthoCornersBaseXY)<<"\n to birds-eye pixels:\n"<<orthoCornersIJ_<<endl<<endl;

	perspectiveTransform(Mat(viewCornersIJ),viewCornersBaseXY_,viewToBase);
	cout<<"project view pixels:\n"<<Mat(viewCornersIJ)<<"\nto base_link co-ordinates:\n"<<viewCornersBaseXY_<<endl<<endl;

	writeMat<Point2f>(viewCornersBaseXY_,"/home/connor/Code/Mobile_Robotics/delta/cameraROI_base_link");


	perspectiveTransform(Mat(viewCornersIJ),orthoCornersIJ_,viewToOrtho);
	cout<<"project view pixels:\n"<<Mat(viewCornersIJ)<<"\nto birds-eye pixels:\n"<<orthoCornersIJ_<<endl<<endl;


	Mat out_;
	warpPerspective(firstImage.t(),out_,viewToOrtho,Size(orthoImageSize.height,orthoImageSize.width));
	out_=out_.t();

	imshow("birdseye",out_);
	waitKey(-1);

	cout<<"got there"<<endl;
    cvDestroyWindow("birdseye");
	return 0;
}
