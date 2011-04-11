#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <opencv/cv.h>

using namespace cv;
using namespace geometry_msgs;

// Opencv kalman filter object
// Documentation at 
//http://opencv.willowgarage.com/documentation/cpp/video_motion_analysis_and_object_tracking.html?highlight=kalman#KalmanFilter
//http://www710.univ-lyon1.fr/~eguillou/documentation/opencv2/classcv_1_1_kalman_filter.html
KalmanFilter kalman;
Mat latest_kalman;

void initKalmanFilter()
{
	// Dimensionality of state vector, measurement vector, and control vector
	int dynam_params = 3;
  int measure_params = 3;
  int control_params = 2;
	kalman = KalmanFilter(dynam_params, measure_params, control_params);
	
	// TODO: find out the correct values for the state transition matrix (A)
//	float A[3][3] = {{1, 0, 0},{0, 1, 0},{0, 0, 1}};
//	kalman.transitionMatrix = Mat(3, 3, CV_32F, A);
	
	// TODO: find out the correct values for the control matrix (B)
//	float B[3][2] = {{1,0},{0,1},{-1,1}};
//	kalman.controlMatrix = Mat(3, 2, CV_32F, B);
	
	// TODO: find out the correct values for process noise (Q)
  setIdentity(kalman.processNoiseCov, Scalar(1e-5));
  
	// Initial state vector
	kalman.statePre = Mat::zeros(3, 1, CV_32F);
	
	// A priori error estimate
	kalman.errorCovPre = Mat::zeros(3, 3, CV_32F);
	kalman.errorCovPre.at<float>(0,0) = 10.0;
	kalman.errorCovPre.at<float>(1,1) = 10.0;
	kalman.errorCovPre.at<float>(2,2) = 10.0;
	
}

// Corrects the kalman filter with a GPS estimate
void updateKalmanFilterGPS(geometry_msgs::Pose gps_estimate, double gps_covariance)
{
	// Generate the measurement (note heading is not used)
	Mat measurement = Mat::zeros(3, 1, CV_32F);
	measurement.at<float>(0,0) = gps_estimate.position.x;
	measurement.at<float>(1,0) = gps_estimate.position.y;
	
	// There is no control input for this measurement
	kalman.controlMatrix = Mat::zeros(3, 1, CV_32F);
	
	// Set the measurement covariance.  The GPS measurement gives an X and a Y coordinate with known variances.  Because the heading is unknown, set it to an arbitrary, large variance.
	kalman.measurementNoiseCov.at<float>(0,0) = gps_covariance;
	kalman.measurementNoiseCov.at<float>(1,1) = gps_covariance;
	kalman.measurementNoiseCov.at<float>(2,2) = 99.9;
	
	// Correct the model based on the latest measurement
	latest_kalman = kalman.correct(measurement);
}

// Updates the kalman filter with an encoder movement
void updateKalmanFilterEnc(int left_delta, int right_delta, double covariance)
{
	// Assume a constant 5% error in encoder values due to wheel slippage
	const float ENCODER_ERROR = 0.05;

	// Set the control input
	Mat kalman.controlMatrix.at<float>(0,0) = left_delta;
	kalman.controlMatrix.at<float>(1,0) = right_delta;
	
	float u[2][1] = {{left_delta},{right_delta1};
	Mat u_mat = Mat(2, 1, CV_32F, u);
	
	//TODO: Set the control covariance
	// kalman.processNoiseCov = ;
	
	// Predict the next state given the wheel encoder movements
	latest_kalman = kalman.predict(u_mat);
}

// Returns the pose estimated by the kalman filter
geometry_msgs::Pose getPositionEstimate()
{
	Mat state = Mat(latest_kalman);
	
	// Load the state from the kalman filter
	geometry_msgs::Pose pose;
	pose.position.x = state.at<float>(0,0);
	pose.position.y = state.at<float>(1,0);
	pose.orientation = tf::createQuaternionMsgFromYaw(state.at<float>(2,0));
}

// Returns the estimated variance of x and y (assumes they are the same, just returns x)
float getPositionVariance()
{
	return( kalman.errorCovPost.at<float>(0,0) );
}

// Returns the estimated error in heading
float getHeadingVariance()
{
	return( kalman.errorCovPost.at<float>(2,2) );
}

// Spits out the kalman filter's estimated position (for debugging)
void kalmanPrintState()
{
	Pose p = getPositionEstimate();
	float pos_var = getPositionVariance();
	float head_var = getHeadingVariance();
	
	ROS_INFO( "Kalman: (%f,%f), heading:%f, pos err:%f, head err:%f",p.position.x, p.position.y, pos_var, head_var);
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pso");
	ros::NodeHandle n;
	ROS_INFO("pso initialized");
	
	while (!ros::ok()){ ros::spinOnce(); }
	
	
	initKalmanFilter();
	ROS_INFO("Kalman filter created");
	
	updateKalmanFilterEnc(0,0,1);
	kalmanPrintState();

	geometry_msgs::Pose p;
	p.orientation = tf::createQuaternionMsgFromYaw(5.0);
	p.position.x = 11.0;
	p.position.y = 12.0;
	//updateKalmanFilterGPS(p,2.0);
	ROS_INFO("Updated with GPS");
	
	//kalmanPrintState();
	//ROS_INFO("Done printing");

	ros::Rate loopTimer(10);
	while(ros::ok()){ros::spinOnce();loopTimer.sleep();}
	return 0;
}
