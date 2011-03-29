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
void ReadMat(Mat_<float> *mat, char* file);

vector<Point3f> LIDARPoints; //aggregator for rod points
vector<Point2f> imagePoints; //aggregator for image points
Mat cameraMat; //intrinsic parameters
Mat distMat; //distortion parameters
tf::TransformListener *tfl;

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
		ros::Subscriber sub_info_;
		ros::Publisher pub_blob_loc;
		sensor_msgs::PointCloud BlobLocations;
		Mat_<float> rvec, tvec;
};

//call this with a point32 to publish the blob's location
void DemoNode::publishBlobLoc(geometry_msgs::Point32 BlobLoc)
{
	BlobLocations.points[0] = BlobLoc;
	sensor_msgs::PointCloud tBlobLoc;
	cout << BlobLoc.x << "," << BlobLoc.y << endl;
	tfl->transformPointCloud("map", BlobLocations, tBlobLoc);
	cout << tBlobLoc.points[0].x << "," << tBlobLoc.points[0].y << endl;
	pub_blob_loc.publish(tBlobLoc);
}
DemoNode::DemoNode():
  it_(nh_)
{

	cout << "reading mats" << endl;
	ReadMat(&rvec, "/home/jinx/ROSCode/delta/Mobile_Robotics/rvec");
	ReadMat(&tvec, "/home/jinx/ROSCode/delta/Mobile_Robotics/tvec");
	cout << "read mats" << endl;
	sub_image_ = it_.subscribe("image", 1, &DemoNode::imageCallback, this);
	sub_info_  = nh_.subscribe<sensor_msgs::CameraInfo>("camera_info",1,&DemoNode::infoCallback,this);
	pub_blob_loc = nh_.advertise<sensor_msgs::PointCloud>("Cam_Cloud", 1);

	BlobLocations.points = vector<geometry_msgs::Point32> (1);
	BlobLocations.header.frame_id = "base_laser1_link";
}
bool cameraCalled = false;
// Callback for CameraInfo (intrinsic parameters)
void DemoNode::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
	const double* K = (msg->K).data();
	const double* D = (msg->D).data();
	Mat(3,3,CV_64F,const_cast<double*>(K)).assignTo(cameraMat,CV_32F);
	Mat(5,1,CV_64F,const_cast<double*>(D)).assignTo(distMat,CV_32F);
	cameraCalled = true;
	//cout<<"I GOT CAMERA INFO!!!!!!!!!!!!\n";
}

void DemoNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if(!cameraCalled) {
	return;
  }
  cout << "huh callback?" << endl;
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


  vector< Point2f > projCenter (3);

  ROS_INFO("CAM: Checking image");

  if(Center.x>0)//Center.x should be 0 if there is no blob
{
  Mat R;
  Rodrigues(rvec, R);
  R.col(1) = R.col(2);
  R.col(2) = tvec;
  Mat projector = (cameraMat * R).inv();

  ROS_INFO("CAM: Point Found");
  //cout << CV_IS_MAT(&CvMat(center)) << "," << CV_IS_MAT(&CvMat(rvec)) << "," << CV_IS_MAT(&CvMat(tvec)) << "," << CV_IS_MAT(&CvMat(cameraMat)) << endl;

   //projectPoints(center, rvec, tvec, cameraMat, (Mat_<float>(5,1) << 0,0,0,0,0), projCenter);

  Mat_<float> spot(3,1);
  spot << (float)Center.x, (float)Center.y,(float)1;
  Mat_<float> result = projector * spot;
  result = (1 / result.at<float>(2,0)) * result;
  //result.col(0) = result.col(0) / result(2,0);
  geometry_msgs::Point32 BlobLoc;

  //BlobLoc.x = projCenter[0].x;
  //BlobLoc.y = projCenter[0].y;
   BlobLoc.x = result(0,0);// / result(0,2) ;
   BlobLoc.y = result(0,1);// / result(0,2);
  publishBlobLoc(BlobLoc);
}
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert to 'bgr8'. Ex was %s", e.what());
  }

}

//reads a mat from the file in ~/.ros
void ReadMat(Mat_<float> *mat, char* file)
{
	FILE* test = fopen("/home/jinx/ROSCode/delta/Mobile_Robotics/deltatest", "w");
	fprintf(test, "%i\n", 1);
	fprintf(test, "%i\n", 1337);
	fclose(test);
	test = fopen("/home/jinx/ROSCode/delta/Mobile_Robotics/deltatest", "r");
	int a,b;
	fscanf(test, "%i", &a);
	fscanf(test, "%i", &b);
	fclose(test);
        cout<<"a,b "<<a<<","<<b<<"\n";

	//cout<<"camout1\n";
        ifstream* infile = new ifstream(file, ifstream::in&ifstream::binary);
	cout << "file opened" << endl;
	int rows, cols, type;
	rows = cols = type = 0;
	(*infile)>>rows;
	cout<<"Scanned Rows, there are/is "<<rows<<" of them\n";
	(*infile)>>cols;
	cout<<"Scanned Columns, there are/is "<<cols<<" of them\n";
	(*infile)>>type;
	cout<<"Scanned Type that number is like "<<type<<" kthxbai\n";
	//cout<<"camout2\n";
	*mat = Mat_<float> (rows, cols); 
	//cout<<"camout3\n";
	unsigned int i, j;
	float f;
	for (i = 0; i < mat->rows; i++)
	{
		//fprintf(mfile,"\n");
		for (j = 0; j < mat->cols; j++)
		{
		        (*infile)>>f;
			((*mat)(i,j)) = f;
			cout << f << ",";
		}     
		cout << endl;
	}
	//cout<<"camout4\n";
	infile->close();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "CameraNode");
	cout << "Camera Initialized" << endl;
	tfl = new tf::TransformListener();
	while (!ros::ok()&&!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce();
	cout << "READY"  << endl;
	DemoNode motion_tracker;
	ROS_INFO("Camera Node Started");
	while(ros::ok()){ros::spinOnce();}
	return 0;
}
