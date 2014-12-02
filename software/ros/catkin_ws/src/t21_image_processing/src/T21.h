/**
 * Receives Laser scans and camera images.
 * Sends features and robot location.
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Pose2D.h"
#include "shared/Feature.h"

#include <sstream>
#include <istream>

class T21 {
  private:
  ros::Publisher feature_pub;
  ros::Publisher location_pub;
  ros::Subscriber laser_sub;
  ros::Subscriber camera_sub;

  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void cameraImageCallback(const sensor_msgs::Image::ConstPtr& msg);

  public:
  T21(ros::NodeHandle node);
  void run();
};

