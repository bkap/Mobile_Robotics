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
		void publishNavLoc(vector<Point2i> NavPoints);
		void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
		void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
		void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
		ros::NodeHandle nh_; // Made this public to access it and subscribe to the LIDAR
	private:
		image_transport::ImageTransport it_;
		image_transport::Subscriber sub_image_;
		ros::Subscriber sub_info_;
		ros::Publisher pub_nav_pts;
		sensor_msgs::PointCloud NavPts;
		Mat_<float> rvec, tvec;
};

//call this with a point32 to publish the blob's location
void DemoNode::publishNavLoc(vector<Point2i> NavPoints)
{
	NavPts.points = transformPts(NavPoints); //this should transform the points into geometry points and it should convert from pixel coordinates to robot coordinates
	sensor_msgs::PointCloud tNavPts;
	tfl->transformPointCloud("map", NavPts, tNavPts);
	pub_nav_pts.publish(tNavPts);
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
	pub_nav_pts = nh_.advertise<sensor_msgs::PointCloud>("Cam_Cloud", 1);
 
	NavPts.header.frame_id = "base_laser1_link";
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

  R.col(2) = tvec;

  ROS_INFO("CAM: Point Found");
  //cout << CV_IS_MAT(&CvMat(center)) << "," << CV_IS_MAT(&CvMat(rvec)) << "," << CV_IS_MAT(&CvMat(tvec)) << "," << CV_IS_MAT(&CvMat(cameraMat)) << endl;

 //projectPoints(center, rvec, tvec, cameraMat, (Mat_<float>(5,1) << 0,0,0,0,0), projCenter);

  Mat_<float> spot(3,1);

  spot << Center.x, Center.y,1;

  Mat_<float> invCameraMat = (cameraMat*R).inv();  

  Mat_<float> result = invCameraMat*spot;
  result.col(0) = result.col(0) / result(2,0);

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

void normalizeColor(cv::Mat& img){
        cv::MatIterator_<cv::Vec<uchar,3> > it=img.begin<cv::Vec<uchar,3> >(),it_end=img.end<cv::Vec<uchar,3> >();
        cv::Vec<uchar,3> p;
        for(;it!=it_end;++it){
                //p=*it; 
                double scale = (*it)[0]+(*it)[1]+(*it)[2];
                //p = scale * *it;// (255.0/scale));
		*it = cv::Vec<uchar,3> (cv::saturate_cast<uchar> (255.0 / scale* (float)(*it)[0]),cv::saturate_cast<uchar>(255.0 / scale * (float)(*it)[1]),cv::saturate_cast<uchar>(255.0 /scale * (float)(*it)[2]));
        }
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
      // Point to the right pixels in the source and destination mats
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

  // Run the line finding algorithm, and draw them on the source image
  HoughLinesP(dst, lines, 1, CV_PI/180, 60, 25, 10 );
  for( int i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }

  // Show the thresholded image
  //cvNamedWindow("thresholded image");
  //imshow("thresholded image",dst);

  // Show the image with lines
  //cvNamedWindow("detected lines");
  //imshow("detected lines",src);
}

//these should be the closest point in the image to the robot.  I am just guessing that the image is 640 by 480 and that the robot is at the bottom of the image, in the middle


void GetNearest(&list<Point2i> Points, &list<Point2i> OrderedPoints, Point2i Target)
{
	double StartDist = 10000;
	double Dist = 10000;
	for (list<Point2i>::iterator it=Points.begin(); it!=Points.end(); i++)
	{
		//distance from start to origin
		TargetDist =  sqrt((Start->x-Target.x)*(Start->x-Target.x)+(Start->y-Target.y)*(Start->y-Target.y))
		Dist = sqrt((it->x-Target.x)*(it->x-Target.x)+(it->y-Target.x)*(it->y-Target.Y))
		if (Dist<TargetDist) Start = it;
	}

	NastyPolyLine.push_back(*Start);
	TempList.erase(Start);
}

#define IMAGE_ORIGIN_X 320
#define IMAGE_ORIGIN_Y 479

list<Point2i> linesToNastyPolyLine(vector<Vec4i> lines)
{
	list<Point2i> NastyPolyLine ();
	list<Point2i> TempList();

	for(i = 0; i<size(lines); i++)
	{
		TempList.push_back(Point2i(lines[i].x1, lines[i].y1));
		TempList.push_back(Point2i(lines[i].x2, lines[i].y2));
	}

	//find nearest to origin
	GetNearest(TempList, NastyPolyLine, Point2i(IMAGE_ORIGIN_X, IMAGE_ORIGIN_Y));

	while(TempList.size()>0)
	{
		GetNearest(TempList, NastyPolyLine, NastyPolyLine.end());
	}
	
	return NastyPolyLine;
}


list<Point2i> cleanNastyPolyLine(list<Point2i> NastyPolyLine, int NumRemaining)
{
	list<Point2i>::iterator leastSignificant;
	double minSignif; //short for minimum significance
	while(NastyPolyLine.size()>NumRemaining)
	{
		minSignif = 10000000; 
		for (list<Point2i>::iterator it=NastyPolyLine.begin(); it!=NastyPolyLine.end(); i++)
		{
			Point2i A = (*(it-1))-(*it);
			Point2i B = (*(it+1))-(*it);
			double angle = asin(A*B/(A.norm()*B.norm));
			double curSignif = fabs(180-angle)*A*B/(A.norm()*B.norm);
			if(curSignif<minSignif)
			{
				minSignif = curSignif;
				leastSignificant = it;
			}
		}
		NastyPolyline.erase(it);
	}	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CameraNode");//
	cout << "Camera Initialized" << endl;
	tfl = new tf::TransformListener();
	while (!ros::ok()&&!tfl->canTransform("map", "odom", ros::Time::now())) ros::spinOnce();
	cout << "READY"  << endl;
	DemoNode motion_tracker;
	ROS_INFO("Camera Node Started");
	while(ros::ok()){ros::spinOnce();}
	return 0;
}
