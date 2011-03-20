#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <geometry_msgs/Twist.h>

#include <eecs376_vision_demo1/lib_demo.h>

using namespace cv;

class DemoNode {
  public:
    DemoNode();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  private:
    ros::NodeHandle nh_;
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
  catch (sensor_msgs::CvBridgeException& e) {

    ROS_ERROR("Could not convert to 'bgr8'. Ex was %s", e.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "eecs376_vision_demo1");
  DemoNode motion_tracker;
  cvNamedWindow("view"); //these cv* calls are need if you want to use cv::imshow anywhere in your program
  cvStartWindowThread();
  ros::spin();
  cvDestroyWindow("view");
}
