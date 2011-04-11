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
	
	/*
	ROS_INFO("statePre is size [%d,%d]",kalman.statePre.rows,kalman.statePre.cols);
	ROS_INFO("errorCovPost is size [%d,%d]",kalman.errorCovPost.rows,kalman.errorCovPost.cols);
	ROS_INFO("measurementMatrix is size [%d,%d]",kalman.measurementMatrix.rows,kalman.measurementMatrix.cols);
	ROS_INFO("processNoiseCov is size [%d,%d]",kalman.processNoiseCov.rows,kalman.processNoiseCov.cols);
	ROS_INFO("measurementNoiseCov is size [%d,%d]",kalman.measurementNoiseCov.rows,kalman.measurementNoiseCov.cols);
	*/
	
	// TODO: find out the correct values for the state transition matrix (A)
	float A[3][3] = {{1, 0, 0},{0, 1, 0},{0, 0, 1}};
	kalman.transitionMatrix = Mat(3, 3, CV_32F, A);
	
	// TODO: find out the correct values for the control matrix (B)
	float B[3][2] = {{1,0},{0,1},{0,0}};
	kalman.controlMatrix = Mat(3, 2, CV_32F, B);
	
	// TODO: find out the correct values for process noise (Q)
  setIdentity(kalman.processNoiseCov, Scalar(1e-3));
  
	// Initial state vector
	kalman.statePre = Mat::zeros(3, 1, CV_32F);
	kalman.statePre.at<float>(0,0)=1;
	kalman.statePre.at<float>(1,0)=2;
	kalman.statePre.at<float>(2,0)=3;
	
	// A priori error estimate
	setIdentity(kalman.measurementMatrix, Scalar(1));
	setIdentity(kalman.processNoiseCov, Scalar(1e-5));
	setIdentity(kalman.measurementNoiseCov, Scalar(.1));
	setIdentity(kalman.errorCovPost,3);
	
	// Start off the filter by simulating zero input
	latest_kalman = Mat(kalman.predict(Mat::zeros(2, 1, CV_32F)));
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
	latest_kalman = Mat(kalman.correct(measurement));
}

// Updates the kalman filter with an encoder movement
void updateKalmanFilterEnc(int left_delta, int right_delta)
{
	// Assume a constant 5% error in encoder values due to wheel slippage
	const float ENCODER_ERROR = 0.05;

	// Set the control input
	float u[2][1] = {{left_delta},{right_delta}};
	Mat u_mat = Mat(2, 1, CV_32F, u);
	
	//TODO: Come up with reliable values for the control covariance
	float var = ENCODER_ERROR * ((float)left_delta + (float)right_delta)/2;
	kalman.processNoiseCov.at<float>(0,0) = var;
	kalman.processNoiseCov.at<float>(1,1) = var;
	kalman.processNoiseCov.at<float>(2,2) = .2;
	
	// Predict the next state given the wheel encoder movements
	latest_kalman = Mat(kalman.predict(u_mat));
}

// Returns the pose estimated by the kalman filter
geometry_msgs::Pose getPositionEstimate()
{
	Mat state = Mat(latest_kalman);
	
	// Load the state from the kalman filter
	geometry_msgs::Pose p;
	p.position.x = state.at<float>(0,0);
	p.position.y = state.at<float>(1,0);
	p.orientation = tf::createQuaternionMsgFromYaw(state.at<float>(2,0));
	return(p);
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
	ROS_INFO( "Kalman: (%.2f,%.2f), head:%.2f, pos var:%f, head var:%f",p.position.x, p.position.y, tf::getYaw(p.orientation), pos_var, head_var);
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pso");
	ros::NodeHandle n;
	ROS_INFO("pso initialized");
	
	while (!ros::ok()){ ros::spinOnce(); }
	
	initKalmanFilter();
	ROS_INFO("Kalman filter created");
	kalmanPrintState();
	
	updateKalmanFilterEnc(1,1);
	ROS_INFO("Updated with encoders at (1,1)");
	kalmanPrintState();

	updateKalmanFilterEnc(3,2);
	ROS_INFO("Updated with encoders (3,2)");
	kalmanPrintState();
	

	geometry_msgs::Pose p;
	p.orientation = tf::createQuaternionMsgFromYaw(5.0);
	p.position.x = 11.0;
	p.position.y = 12.0;
	
	for( int i=0; i<5; i++ )
	{
		updateKalmanFilterGPS(p,2.0);
		ROS_INFO("Updated with GPS");
		kalmanPrintState();
	}
	//ROS_INFO("Done printing");

	ros::Rate loopTimer(10);
	while(ros::ok()){ros::spinOnce();loopTimer.sleep();}
	return 0;
}
