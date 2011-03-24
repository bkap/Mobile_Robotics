//#include <camera_info_manager/camera_info_manager.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <eecs376_vision_demo1/lib_demo.h>

using namespace cv;
using namespace std;

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
    static void info(const sensor_msgs::CameraInfo msg);
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    ros::NodeHandle nh_; // Made this public to access it and subscribe to the LIDAR
  private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image_;
    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_info_;
    image_transport::Publisher image_pub_;
};

DemoNode::DemoNode():
  it_(nh_)
{
  sub_image_ = it_.subscribe("image", 1, &DemoNode::imageCallback, this);
  sub_lidar_ = nh_.subscribe<sensor_msgs::LaserScan>("lidar",1,&DemoNode::lidarCallback,this);  
  sub_info_  = nh_.subscribe<sensor_msgs::CameraInfo>("camera_info",1,&DemoNode::infoCallback,this);
  image_pub_ = it_.advertise("demo_image", 1);
}
// Callback for CameraInfo (intrinsic parameters)
void DemoNode::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
//	const double* K = (msg->K).data();
//	const double* D = (msg->D).data();
//	Mat(3,3,CV_64F,&K).convertTo(cameraMat,CV_32F,true);
//	Mat(5,1,CV_64F,&D).convertTo(distMat,CV_32F,true);
cout<<"got camera\n";
	
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
    cv::Point2f center (Center.x, Center.y);
    if(center.x>0 && last_scan_valid){
	//print id,i,j,r,theta,x,y as per assignment
	cout<< msg->header.seq<<"\t"<<center.x<<"\t"<<center.y<<"\t"<<norm(lastValidLIDARPoint)<<"\t"<<atan2(lastValidLIDARPoint.y,lastValidLIDARPoint.x);
	cout<<"\t"<<lastValidLIDARPoint.x<<"\t"<<lastValidLIDARPoint.y<<endl;

	imagePoints.push_back(center);
	LIDARPoints.push_back(Point3f(lastValidLIDARPoint.x,lastValidLIDARPoint.y,0));
    }
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

// from http://blog.weisu.org/2007/11/opencv-print-matrix.html
void PrintMat(CvMat *A)
{
    int i, j;
    for (i = 0; i < A->rows; i++)
    {
        printf("\n");
        switch (CV_MAT_DEPTH(A->type))
        {
            case CV_32F:
            case CV_64F:
                for (j = 0; j < A->cols; j++)
                printf ("%8.3f ", (float)cvGetReal2D(A, i, j));
                break;
            case CV_8U:
            case CV_16U:
                for(j = 0; j < A->cols; j++)
                printf ("%6d",(int)cvGetReal2D(A, i, j));
                break;
            default:
                break;
        }
    }
    printf("\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "eecs376_vision_demo1");
  DemoNode motion_tracker;
  //ros::Subscriber lidar_sub = motion_tracker.nh_.subscribe<sensor_msgs::LaserScan>("lidar", 1, lidarCallback); // Subscribe to the LIDAR scan
  //ros::Subscriber info_sub  = motion_tracker.nh_.subscribe<sensor_msgs::CameraInfo>("camera_info",1,infoCallback); //Subscribe to the camera info
  cvNamedWindow("view"); //these cv* calls are need if you want to use cv::imshow anywhere in your program
  cvStartWindowThread();
  ROS_INFO("Calibration procedure started");
 // ros::Rate naptime(75);

  //while(imagePoints.size()<100)
  //{
  //      naptime.sleep();
  //	ros::spinOnce();
  //}
	ros::spin();
	Mat iPoints_ = Mat(imagePoints); //matrix of points in image-space
	Mat wPoints_ = Mat(LIDARPoints); //matrix of points in world-space
	iPoints_.convertTo(iPoints_,CV_64F);
	wPoints_.convertTo(wPoints_,CV_64F);
	
	CvMat iPoints = iPoints_;
	CvMat wPoints = wPoints_;

	//there's probably going to be problems since the centroids were calculated from images which had already undergone some manner of undistortion/rectification
	Mat rvec_ = (Mat_<double>(3,1) << 0.0,0.0,0.0);
	Mat tvec_ = (Mat_<double>(3,1) << 0.0,0.0,0.0);
	CvMat rvec = rvec_;//Mat(3,1,CV_64F); //extrinsic parameter rotation matrix
	CvMat tvec = tvec_;///Mat(3,1,CV_64F); //extrinsic parameter translation matrix

if(imagePoints.size()<20)	return 1;

        cameraMat = (Mat_<double>(3,3) << 532.18899999999996, 0.0, 330.678, 0.0, 531.28700000000003, 260.733, 0.0, 0.0, 1.0);
        distMat = (Mat_<double>(5,1) <<  -0.34610800000000003, 0.17599799999999999, 0.0013294100000000001, 0.0023169800000000002, 0.0);
	CvMat cvCameraMat =cameraMat;
	CvMat cvDistMat = distMat;
        
        cout<<"CV_IS_MAT\tiPoints\twPoints\trvec\ttvec\tcvCameraMat\tcvDistMat\n";
	cout<<"\t\t"<<CV_IS_MAT(&iPoints)<<"\t"<<CV_IS_MAT(&wPoints)<<"\t"<<CV_IS_MAT(&rvec)<<"\t"<<CV_IS_MAT(&tvec)<<"\t"<<CV_IS_MAT(&cvCameraMat)<<"\t\t"<<CV_IS_MAT(&cvDistMat)<<"\n";

	cvFindExtrinsicCameraParams2(&wPoints,&iPoints,&cvCameraMat,NULL,&rvec,&tvec);
	cout<<endl;
	cout<<"rotations:";
	PrintMat(&rvec);
	cout<<"\ntranslations:";
	PrintMat(&tvec);
	cout<<endl;

	rvec_ = Mat(&rvec);
	tvec_ = Mat(&tvec);

	Mat R;
	Rodrigues(rvec_, R);
	R.col(1) = R.col(2);
	R.col(2) = tvec_;

	Mat projector = (cameraMat * R).inv();
	
	/*
	reverse projection should be robotCoordinateFramePoint = projector * imagePoint
	(x,y,1) = projector * (u v 1)
	still need to verify by comparison of projected blob centroids with corresponding lidar pings
	*/
	cout<<"got there"<<endl;
    cvDestroyWindow("view");
return 0;
}
