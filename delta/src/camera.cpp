//#include <camera_info_manager/camera_info_manager.h>
#include <ros/ros.h>
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
#include <stdio.h>
using namespace cv;
using namespace std;
void ReadMat(Mat *mat, char* file);
bool last_scan_valid; // This is set to true by the LIDAR callback if it detects a plausible location for the rod.
// Also declare a point datatype here to hold the location of the rod as detected by LIDAR
Point2f lastValidLIDARPoint; //set this the x,y coordinate of any rod found
vector<Point3f> LIDARPoints; //aggregator for rod points
vector<Point2f> imagePoints; //aggregator for image points
Mat cameraMat; //intrinsic parameters
Mat distMat; //distortion parameters
class DemoNode {
  public:
    DemoNode();
    //static void info(const sensor_msgs::CameraInfo msg);
    void publishBlobLoc(geometry_msgs::Point32 BlobLoc);
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    ros::NodeHandle nh_; // Made this public to access it and subscribe to the LIDAR
  private:
     image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image_;
    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_info_;
    ros::Publisher pub_blob_loc;
    sensor_msgs::PointCloud BlobLocations;
    Mat rvec, tvec;
};

//call this with a point32 to publish the blob's location
void DemoNode::publishBlobLoc(geometry_msgs::Point32 BlobLoc)
{
   BlobLocations.points[0] = BlobLoc;
   pub_blob_loc.publish(BlobLoc);
}
DemoNode::DemoNode():
  it_(nh_)
{
  sub_image_ = it_.subscribe("image", 1, &DemoNode::imageCallback, this);
  sub_lidar_ = nh_.subscribe<sensor_msgs::LaserScan>("lidar",1,&DemoNode::lidarCallback,this);  
  sub_info_  = nh_.subscribe<sensor_msgs::CameraInfo>("camera_info",1,&DemoNode::infoCallback,this);
  pub_blob_loc = nh_.advertise<sensor_msgs::PointCloud>("Cam_Cloud", 1);

  BlobLocations.points = vector<geometry_msgs::Point32> (1);

  ReadMat(&rvec, "rvec");
  ReadMat(&tvec, "tvec");
}
// Callback for CameraInfo (intrinsic parameters)
void DemoNode::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
	const double* K = (msg->K).data();
	const double* D = (msg->D).data();
	Mat(3,3,CV_64F,const_cast<double*>(K)).assignTo(cameraMat,CV_32F);
	Mat(5,1,CV_64F,const_cast<double*>(D)).assignTo(distMat,CV_32F);
       //cout<<"I GOT CAMERA INFO!!!!!!!!!!!!\n";
}


// Callback triggered whenever you receive a laser scan
void DemoNode::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  sensor_msgs::LaserScan scan = *msg;

  // First, filter the laser scan to remove random bad pings.  Search through the laser scan, and pick out all points with max range.  Replace these points by the average of the two points on either side of the bad point.
  int num_points = scan.ranges.size();
  int num_filtered = 0;

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

  int peakIndex;
  double maxVal = -DBL_MAX;
  double val = 0;

    for (int i=1;i<num_points-1;i++){
	    val = (0.5 * scan.ranges[i-1] - scan.ranges[i] + 0.5*scan.ranges[i+1])/scan.ranges[i];
	    if(val>maxVal){
		    maxVal = val;
		    peakIndex = i;
	    }
    }

    double theta = 3.141592653589793238462643383279 * (double)(peakIndex-90) / 180.0;
    double dist = scan.ranges[peakIndex];

if(maxVal > 0.8){	//threshold
	last_scan_valid = true;
	lastValidLIDARPoint.x = dist*cos(theta);
	lastValidLIDARPoint.y = dist*sin(theta);
}
  ROS_INFO("LIDAR scan received. Smoothed out %d bad points out of %d maxVal: %f",num_filtered,num_points,maxVal);
}

void DemoNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::CvBridge bridge;
  cv::Mat image;
  cv::Mat output;
  try
  {
    image = cv::Mat(bridge.imgMsgToCv(msg, "bgr8"));

  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'. E was %s", msg->encoding.c_str(), e.what());
  }
  try {
    normalizeColors(image, output);
    CvPoint2D64f Center = blobfind(image, output);
    if(last_scan_valid)
    {
	//print id,i,j,r,theta,x,y as per assignment
	//cout<< msg->header.seq<<"\t"<<center.x<<"\t"<<center.y<<"\t"<<norm(lastValidLIDARPoint)<<"\t"<<atan2(lastValidLIDARPoint.y,lastValidLIDARPoint.x);
	//cout<<"\t"<<lastValidLIDARPoint.x<<"\t"<<lastValidLIDARPoint.y<<endl;
	
  Mat center (1, 3, CV_64);
  center(0,0) = Center.x;
  center(0,1) = Center.y;
  center(0,2) = 0;
  vector<Point3f> projCenter (1);
  
  projectPoints(center, rvec, tvec, cameraMat, distMat, projCenter);  

  geometry_msgs::Point32 BlobLoc;

  BlobLoc.x = projCenter[0].x;
  BlobLoc.y = projCenter[0].y;
  BlobLoc.z = 0;

  publishBlobLoc(BlobLoc);}
 //   cv::imshow("view", output);
 //   findLines(image, output);
 //   IplImage temp = output;
 //   image_pub_.publish(bridge.cvToImgMsg(&temp, "bgr8"));
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert to 'bgr8'. Ex was %s", e.what());
  }
}

//reads a mat from the file in ~/.ros
void ReadMat(Mat *mat, char* file)
{
    FILE* mfile = fopen(file, "r");
    int rows, cols, type;
    fscanf(mfile, "%i", &rows);
    fscanf(mfile, "%i", &cols);
    fscanf(mfile, "%i", &type);
    
    mat = new Mat (rows, cols, type); 

    unsigned int i, j;
    float f;
    for (i = 0; i < mat->rows; i++)
    {
        fprintf(mfile,"\n");
        for (j = 0; j < mat->cols; j++)
        {
		fscanf (mfile,"%8.3f", &f);
		((*mat)(i,j)) = f;
	}
        
    }
    fclose(mfile);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "CameraNode");
  DemoNode motion_tracker;
  ROS_INFO("Camera Node Started");
  ros::spin();
return 0;
}
