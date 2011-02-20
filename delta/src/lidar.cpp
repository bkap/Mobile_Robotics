#include<ros/ros.h>
#include<tf/transform_datatypes.h> // for tf::getYaw
#include<tf/transform_listener.h> // for the TransformListener class that abstracts away a lot of tf
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h> //for the laser projector class
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

sensor_msgs::PointCloud last_scan_cloud;
sensor_msgs::PointCloud last_map_cloud;
sensor_msgs::LaserScan last_scan;
laser_geometry::LaserProjection projector;
tf::TransformListener *tfl;
ros::Publisher P;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  last_scan = *scan; // save the last scan... just incase you want it in not Point Cloud format??
  projector.projectLaser(*scan, last_scan_cloud); // Do polar coordinate to cartesian coordinate conversion on the current scan. Save the results in the last_scan_cloud.
  tfl->transformPointCloud("map", last_scan_cloud, last_map_cloud); //transform the cloud into the map frame
  ROS_INFO("Successfully transformed %d points", last_map_cloud.points.size());
  P.publish(last_map_cloud);
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"lidar_listener");//name of this node
  ros::NodeHandle n;
  tfl = new tf::TransformListener();
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(n, "scan", 1); //Subscriber object, but now we want to apply filters to it, so it's not just a standard ros::Subscriber
  tf::MessageFilter<sensor_msgs::LaserScan> laser_filter(laser_sub, *tfl, "map", 1); //Make a MessageFilter for the laser_sub that will only allow a message through when we can transform it into the given frame (in this case, map)
  laser_filter.registerCallback(boost::bind(laserCallback, _1)); //When a message passes through the laser_filter, we want it to call laserCallback
  P = n.advertise<sensor_msgs::PointCloud>("LIDAR_Cloud", 1);
  ros::spin(); //spin until ctrl-C or otherwise shutdown
  delete tfl;
  return 0; // this code will only get here if this node was told to shut down, which is
  // reflected in ros::ok() is false 
}
