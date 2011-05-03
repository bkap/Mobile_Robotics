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


/*	Connor Balin

	The camera calibration procedure uses identified correspondances between the LIDAR data and camera data to compute several transformation matrices.
	The primary transformation calculated (and the only one used elsewhere) is the homography between the image plane and base_link coordinate system.
	Additional transformations include the plan-view transform using view image specifications (physical pixel size and image physical extents) specified as parameters 		to the calibration procedure and a transform from the birdseye image plane to base_link co-ordinates

	Use of openCV's homography matrices and associated transform functions obviates the need for use of homogeneous co-ordinates, which are handled internally, and 	implements RANSAC in the determination of the transforms.
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
Mat firstImage;			//first image pulled from the callback

/*
	 Function reads an OpenCV Mat from disk
*/
template <typename T>
void readMat(cv::Mat_<T>& mat, char* file){
	ifstream* infile = new ifstream(file,ifstream::in|ifstream::binary);
	int rows = 0,cols = 0,type=0,size=0;
	(*infile)>>rows;
	(*infile)>>cols;
	(*infile)>>type;
	(*infile)>>size;
	char* data = new char[size];
	infile->read(data,size);
	infile->close();

	int sizes[2] = {rows,cols};
	Mat_<T> temp = Mat(2,sizes,type,data);
	temp.copyTo(mat);
	delete[] data;
}

/*
	Function writes an OpenCV Mat to disk
*/
template <typename T>
void writeMat(cv::Mat_<T>& mat, char* file){
	ofstream* ofile = new ofstream(file,ofstream::out|ofstream::binary);
	(*ofile)<<mat.rows<<" ";
	(*ofile)<<mat.cols<<" ";
	(*ofile)<<mat.type()<<" ";
	(*ofile)<<mat.rows*mat.cols*mat.elemSize();
	
	ofile->write((char*) mat.data,mat.rows*mat.cols*mat.elemSize());
	ofile->close();
}

/*
	function prints an OpenCV mat for debugging
*/
template<typename T>
void printMat(cv::Mat_<T>& mat, ostream stream = cout){
	for(int i=0;i<mat.rows;i++){
		for(int j=0;j<mat.cols;j++){
			stream << mat(i,j);
			stream << (j+1==mat.cols ? ";\n":" , ");
		}
	} 
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

// Class constructor initializes data structures
DemoNode::DemoNode():
  it_(nh_)
{
	// Subscribe to the front camera and LIDAR, publish an image for visualization/debugging.
  sub_image_ = it_.subscribe("image", 1, &DemoNode::imageCallback, this);
  sub_lidar_ = nh_.subscribe<sensor_msgs::LaserScan>("lidar",1,&DemoNode::lidarCallback,this);
  image_pub_ = it_.advertise("demo_image", 1);
}

/*
 * This function filters unwanted discontinuities from LIDAR scans.
 * For each point in the scan, if it is near max range, it is replaced by
 * the average of the two neighboring points.  This should reduce random
 * errors in the LIDAR scans due to some pings getting lost (assuming
 * that lost pings are assigned the value at max range)
 */
int filterLIDAR(sensor_msgs::LaserScan& scan){
  int num_filtered = 0;
  int num_points = scan.ranges.size();
  
  for( int i=1; i<num_points-1; i++ )
  {
    // This pixel is a candidate for filtering if it is at max range
    if( scan.ranges[i]>=scan.range_max-1.5 )
    {
    	// Replace it with the average of its two neighbors
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

/*
 * Given a LIDAR scan, this function returns true if the calibration
 * rod could plausibly be explained by this scan.
 */
bool findRod(sensor_msgs::LaserScan& scan)
{
	float rodSepThresh  = 0.4; //min dist of rod from surroundings
	float rodDispThresh = 0.05;//max radial diff between rod points
	float rodWidthThresh= 3;   //max pings of a rod
	float rodDistThresh = 4;   //max distance to rod
	bool maybeRod = false;
	int rodStart = -1;
	float minr = 1000;
	float maxr = 0;

	last_scan_valid = false;

	// Iterate through each point in the scan to find the rod
	for(unsigned int i=1;i<scan.ranges.size()-1;i++)
	{
		// You just saw a jump in distance that could plausibly signify the start of the rod
		if(!maybeRod && (scan.ranges[i-1] - scan.ranges[i])>rodSepThresh)
		{
			maybeRod = true;
			// Record the current postion as the start of the rod
			rodStart = i;
			minr = scan.ranges[i];
			maxr = scan.ranges[i];
		}
		// If you think you might be looking at the rod
		if(maybeRod)
		{
			// Keep track of the minimum and maximum distance you've seen
			minr = scan.ranges[i] < minr? scan.ranges[i]:minr;
			maxr = scan.ranges[i] > maxr? scan.ranges[i]:maxr;
			
			// You just saw a jump in distance that could plausibly signify the end of the rod
			if(scan.ranges[i+1] - scan.ranges[i] > rodSepThresh)
			{
				// You did not detect a rod if the sequence of points you found was too wide, too
				// far away, or had too steep of an angle.
				if(i-rodStart >= rodWidthThresh || maxr - minr > rodDispThresh || maxr > rodDistThresh)
				{
					maybeRod = false;
				}
				// The sequence of points that you saw do look like the calibration rod
				else
				{
					// Distance to the rod
					float r = scan.ranges[i];
					// Thickness of the rod
					float t = CV_PI * ( ((float) (rodStart+i))/360.0 -0.5);
					
					// Signal to the main program that you found the rod.
					last_scan_valid = true;
					rodLocation.x = r*cos(t);
					rodLocation.y = r*sin(t);
					cout<<"rod at: "<<r<<" m "<<t*180/CV_PI<<endl;
					
					// start over
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
void DemoNode::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	sensor_msgs::LaserScan scan = *msg;
	ROS_INFO("lidar callback starting");

	int num_filtered = filterLIDAR(scan);	//smooth out any spikes to max range and record number of fixed points
	
	bool foundRod = findRod(scan);	// Check for detection of the calibration rod in this scan

	if(foundRod){
		ROS_INFO("LIDAR detected rod after smoothing %d bad points",num_filtered);
	}
}

/*
	The image callback
*/
void DemoNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_INFO("image callback starting");
  	static sensor_msgs::CvBridge bridge;
  	cv::Mat image;
  	cv::Mat output;
  	static bool first = true;
  	try
  	{
		image = cv::Mat(bridge.imgMsgToCv(msg,"bgr8"));
	
		/* code for flipping from harlie bag */
		// used a hack for flipping because accounting for all possible permutations of orientation made the transforms inconvenient  
    		//Mat im_ = cv::Mat(bridge.imgMsgToCv(msg, "bgr8"));
    		//flip(im_,image,-1);
	  
    		lastImage = image.clone();

		if(first)	firstImage = image.clone();
		first = false;
		ROS_INFO("converted image:\n\trxc:\t%dx%d\n\tdepth:\t%d",image.rows,image.cols,image.depth());
	}
	catch(...) //catch everything incase cvBridge messes up extra hard
	{
		ROS_ERROR("failed to convert");
	}
	try
	{
		// Detect orange blobs
		ROS_INFO("Image callback finding blob");
		normalizeColors(image, output);
		ROS_INFO("normalized colors");
		CvPoint2D64f Center = blobfind(image, output);
		ROS_INFO("Image callback checked for blob");
		cv::Point2f center (Center.x, Center.y);
		
		// If you detected a blob, AND the last lidar scan detected the calibration rod
		if(center.x>0 && last_scan_valid)
		{
			// You found a valid calibration point.
			ROS_INFO("calbiration point acquired");
			
			// Store the latest point
			viewIJ.push_back(Point2f(center.y, center.x));	//image points are in (i,j) instead of (x,y) so co-ordinates are reversed
			baseXY.push_back(Point2f(rodLocation.x,rodLocation.y));
			birdsIJ.push_back(Point2f(
			orthoImageSize.height - rodLocation.x * ppm,
			(orthoImageSize.width-1)/2 - rodLocation.y * ppm));
		}
		/*	code for displaying plan-view transform if transforms are already around
		warpPerspective(image.t(),output,viewToOrthoInv,Size(orthoImageSize.height,orthoImageSize.width),WARP_INVERSE_MAP);
		output = output.t();
		*/
		
		// Publish the image for visualization/debugging
		IplImage temp = output;
		image_pub_.publish(bridge.cvToImgMsg(&temp, "bgr8"));
		ROS_INFO("ending blob finding");
	}
	catch (...)	//continue to catch everything incase cvBridge messes up extra hard
	{
	ROS_ERROR("failed for some reason");
	}
	ROS_INFO("ending image callback");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robust_calibration");
	DemoNode motion_tracker;
	cvNamedWindow("birdseye",CV_WINDOW_AUTOSIZE); //these cv* calls are need if you want to use cv::imshow anywhere in your program
	//cvStartWindowThread();
	ROS_INFO("Calibration procedure started");
	ros::Rate naptime(75);

	// Wait for the callbacks to run until you've acquired enough calibration points
	while(ros::ok() && viewIJ.size()<120)
	{
		naptime.sleep();
		ros::spinOnce();
	}
  
	ROS_INFO("beginning calibration");

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
	//transform between camera pixel co-ordinates and base_link (x,y)
	Mat_<double> viewToBaseInv = findHomography(baseXY_,viewIJ_,mask1,RANSAC,10);	

	//transform between camera and birds-eye pixel co-ordinates
	Mat_<double> viewToOrthoInv= findHomography(birdsIJ_,viewIJ_,mask2,RANSAC,10);	

	//transform between birds-eye pixel co-ordinates and base_link (x,y)
	Mat_<double> orthoToBaseInv= findHomography(baseXY_,birdsIJ_,mask3,RANSAC,10);	

	//transforms going the other way
	Mat_<double> orthoToBase,viewToBase,viewToOrtho;
	invert(orthoToBaseInv,orthoToBase);
	invert(viewToBaseInv,viewToBase); 
	invert(viewToOrthoInv,viewToOrtho);
	
	// Write the transformation matrix to file
	writeMat(viewToBase,"viewToBase");
	//writeMat(viewToOrthoInv,"viewToOrthoInv");
	//writeMat(orthoToBaseInv,"baseToOrthoInv");

	/* The following code is for debugging, to verify that the transformation was successful */

	cout<<"viewToBase inliers "<<countNonZero(Mat(mask1))<<"\nviewToOrtho inliers "<<countNonZero(Mat(mask2))<<"\northoToBase inliers "<<countNonZero(Mat(mask3))<<endl;
	cout<<"Birds eye image size width,height: "<<orthoImageSize.width<<","<<orthoImageSize.height<<endl;

	perspectiveTransform(Mat(orthoCornersBaseXY),orthoCornersIJ_,orthoToBaseInv);
	//cout<< "project base_link co-ordinates:\n"<<Mat(orthoCornersBaseXY)<<"\n to birds-eye pixels:\n"<<orthoCornersIJ_<<endl<<endl;

	perspectiveTransform(Mat(viewCornersIJ),viewCornersBaseXY_,viewToBase);
	//cout<<"project view pixels:\n"<<Mat(viewCornersIJ)<<"\nto base_link co-ordinates:\n"<<viewCornersBaseXY_<<endl<<endl;

	writeMat(viewCornersBaseXY_,"cameraROI_base_link");

	perspectiveTransform(Mat(viewCornersIJ),orthoCornersIJ_,viewToOrtho);
	//cout<<"project view pixels:\n"<<Mat(viewCornersIJ)<<"\nto birds-eye pixels:\n"<<orthoCornersIJ_<<endl<<endl;

	//display the first image after applying the plan-view transform to verify transform corrrectness
	Mat out_;
	warpPerspective(firstImage.t(),out_,viewToOrtho,Size(orthoImageSize.height,orthoImageSize.width));
	out_=out_.t();

	imshow("birdseye",out_);
	waitKey(-1);

	cout<<"got there"<<endl;
	cvDestroyWindow("birdseye");
	return 0;
}
