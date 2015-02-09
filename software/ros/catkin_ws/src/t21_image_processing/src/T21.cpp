#include "T21.h"

T21::T21(ros::NodeHandle node) {
  feature_pub = node.advertise<shared::Feature>("t21_feature", 100);
  location_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("t21_robot_location", 100);
  laser_sub = node.subscribe("laser_scan", 100, &T21::laserScanCallback, this);
  camera_sub = node.subscribe("camera_image", 10, &T21::cameraImageCallback, this);
}

void T21::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
// TODO
}

void T21::cameraImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
// TODO
}

void T21::run() {
  ros::spin();
}

