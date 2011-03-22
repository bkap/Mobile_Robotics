#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <eecs376_vision_demo1/lib_demo.h>

using namespace cv;

Point2f lastValidLIDARPoint;

bool last_scan_valid; // This is set to true by the LIDAR callback if it detects a plausible location for the rod.
// Also declare a point datatype here to hold the location of the rod as detected by LIDAR

class DemoNode {
  public:
    DemoNode();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    ros::NodeHandle nh_; // Made this public to access it and subscribe to the LIDAR
  private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
    image_transport::Publisher image_pub_;
};

DemoNode::DemoNode():
  it_(nh_)
{
  sub_ = it_.subscribe("image", 1, &DemoNode::imageCallback, this);
  image_pub_ = it_.advertise("demo_image", 1);
}

// Callback triggered whenever you receive a laser scan
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
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
  double MaxVal = -DBL_MAX;//set to an arbitrarily low value  

  //convolution of lidar ranges with [1/2, -1, 1/2]
  //we don't need to store the convolution long term, just each individual value.
  
  double Val = 0;
  
  for(int i =1; i<num_points-1; i++)
  {
      Val = (scan.ranges[i-1]*.5+scan.ranges[i]+scan.ranges[i+1]*.5)/scan.ranges[i];
      if(Val>MaxVal)
      {
           MaxVal = Val;
           peakIndex = i;
      }
  }

  double theta = (double)(i-90);
  double dist = scan.ranges[i];

  lastValidLIDARPoint.x = dist*cos(theta);
  lastValidLIDARPoint.y = dist*sin(theta);
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
    cv::Point2f center (Center.x, Center.y);
    cv::imshow("view", output);
    findLines(image, output);
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
  ros::init(argc, argv, "eecs376_vision_demo1");
  DemoNode motion_tracker;
  ros::Subscriber lidar_sub = motion_tracker.nh_.subscribe<sensor_msgs::LaserScan>("base_laser1_scan", 1, laserCallback); // Subscribe to the LIDAR scan
  cvNamedWindow("view"); //these cv* calls are need if you want to use cv::imshow anywhere in your program
  cvStartWindowThread();
  ROS_INFO("Calibration procedure started");
  ros::spin();
  cvDestroyWindow("view");
}
