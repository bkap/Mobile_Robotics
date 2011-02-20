#include "command_publisher.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gemoetry_msgs/PoseStamped.h>
#include "cv.h"
#include <list>


using namespace cv;
/**assume orientation and resolution are the same*/
Mat& getMap(OccupancyGrid grid) {
	Mat<bool> m = Mat<int>(grid.info.width, grid.info.height);
	for(int i = 0; i < grid.info.height; i ++) {
		for(int j = 0; j < grid.info.width; j++) {
			m(i,j) = (grid.data(i * grid.info.width + j) > 10);
		}
	}
	return m;

}

list<Pose> bugAlgorithm(Mat& cspace, Point_ dest, PoseStamped start) {
	list<Pose> path =  list<Point_>();
	//go forward until the space 75 cm to the right of the robot is clear. FInd
	//location
	double heading = tf::GetYaw(start.pose.orientation);
	double x = start.pose.x;
	double y = start.pose.y;
	double wallx, wally;
	bool avoiding = false;
	//head forward until you can turn or until you hit the entrance
	while(fabs(x - dest.x) >= 0.03 || fabs(y - dest.y) >= 0.03) {
		x += 0.05 * cos(heading);
		y += 0.05 * sin(heading);
		
		double distance_to_check = avoiding ? 0.6 : 0.75;
		wallx = x + distance_to_check * cos(heading - 3.14159/2);
		wally = y + distance_to_check * sin(heading - 3.14159/2);
		int grid_wall_x = (int)(wallx/CSPACE_RESOLUTION);
		int grid_wall_y = (int)(wally/CSPACE_RESOLUTION);
		int grid_x = (int)(x/CSPACE_RESOLUTION);
		int grid_y = (int)(y/CSPACE_RESOLUTION);
		if(!map(grid_wall_x, grid_wall_y)) {
			if(!avoiding) {
				//this means that we need to turn
				path.push_back(Pose(x,y,heading));
				//now move us around the circle
				x += 0.75 * cos(heading);
				y += 0.75 * sin(heading);
				heading -= 3.14159/2;
				x += 0.75 * cos(heading);
				y += 0.75 * sin(heading);
				path.push-back(Pose(x,y,heading));
			} else {
				//this means we need to readjust to go back 2 feet
				path.push_back(Pose(x,y,heading));
				x = wallx + 1.5 * cos(heading);
				y = wally + 1.5 * cos(heading);
				path.push_back(Pose(x,y,heading));
				avoiding=false;
			}
		} else if(map(grid_x, grid_y)) {
			//we have an obstacle. Back up 1.5 m and swerve around
		
			//TODO: check to see if we are in fact traveling 1.5m before
			//turning
			double prev_turn_x = x - 1.5 * cos(heading);
			double prev_turn_y = y - 1.5 * sin(heading);
			path.push_back(Pose(prev_turn_x, prev_turn_y,heading));
			
			//now get the final position
			x += 0.6 * cos(heading + 3.14159/2);
			y += 0.6 * sin(heading + 3.14159/2);
			path.push_back(Pose(x,y,heading));
			avoiding = true;
		}
	}
	path.push_back(Pose(x,y,heading));
	return path;
}

