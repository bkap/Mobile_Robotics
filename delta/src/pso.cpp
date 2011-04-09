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

void initKalmanFilter()
{
	// Dimensionality of state vector, measurement vector, and control vector
	int dynam_params = 3;
  int measure_params = 3;
  int control_params = 2;
	kalman = KalmanFilter(dynam_params, measure_params, control_params);
	
	// TODO: find out the correct values for the state transition matrix
	// State transition matrix (size 3x2, 32-bit float)
	float m[3][3] = {{1, 0, 0},{0, 1, 0},{0, 0, 1}};
	kalman.transitionMatrix = Mat(3, 1, CV_32F,m);
	
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
	// Generate the measurement
	Mat measurement = Mat::zeros(3, 1, CV_32F);
	measurement.at<float>(0,0) = gps_estimate.position.x;
	measurement.at<float>(1,0) = gps_estimate.position.y;
	
	// There is no control input for this measurement
	kalman.controlMatrix = Mat::zeros(3, 1, CV_32F);
	
	// Set the measurement covariance.  The GPS measurement gives an X and a Y coordinate with known variances.  Because the heading is unknown, set it to an arbitrary value with a large variance.
	kalman.measurementNoiseCov.at<float>(0,0) = gps_covariance;
	kalman.measurementNoiseCov.at<float>(1,1) = gps_covariance;
	kalman.measurementNoiseCov.at<float>(2,2) = 9999.9;
	
	// Update the model
	kalman.correct(measurement);
}

// Updates the kalman filter with an encoder movement
void updateKalmanFilterEnc(int left_delta, int right_delta, double covariance)
{
	// Assume a constant 5% error in encoder values due to wheel slippage
	const double ENCODER_ERROR = 0.05;

	// Set the control input
	kalman.controlMatrix.at<float>(0,0) = left_delta;
	kalman.controlMatrix.at<float>(1,0) = right_delta;
	
	//TODO: Set the control covariance
	
	kalman.predict();
}

// Returns the pose estimated by the kalman filter
geometry_msgs::Pose getPositionEstimate()
{
	Mat state = kalman.statePost;
	
	// Load the state from the kalman filter
	geometry_msgs::Pose pose;
	pose.position.x = state.at<float>(0,0);
	pose.position.y = state.at<float>(1,0);
	pose.orientation = tf::createQuaternionMsgFromYaw(state.at<float>(2,0));
}

// Returns a point structure containing the estimated variance of x and y
double getPositionVariance()
{
	return( kalman.errorCovPost.at<float>(0,0) );
}

// Returns the estimated error in heading
double getHeadingVariance()
{
	return( kalman.errorCovPost.at<float>(2,2) );
}

int main()
{


}
