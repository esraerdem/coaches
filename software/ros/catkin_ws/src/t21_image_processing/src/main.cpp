/**
 * Receives Laser scans and camera images.
 * Sends features and robot location.
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"

#include <sstream>
#include <istream>

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
// TODO
}

void cameraImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
// TODO
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "t21");

  ros::NodeHandle node;

  ros::Publisher feature_pub = node.advertise<shared::Feature>("t21_features", 100);
  ros::Publisher location_pub = node.advertise<geometry_msgs::Pose2D>("robot_location", 100);
  ros::Subscriber laser_sub = node.subscribe("laser_scan", 100, laserScanCallback);
  ros::Subscriber camera_sub = node.subscribe("camera_image", 10, cameraImageCallback);

  ros::spin();

  return 0;
}

