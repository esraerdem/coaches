/**
 * Receives Laser scans and camera images.
 * Sends features and robot location.
 */

#include "ros/ros.h"
#include "shared/Feature.h"
#include "t31_multimodal_hri/HRIActuation.h"
#include <sstream>
#include <istream>

class T31 {
  private:
  ros::Publisher feature_pub;
  ros::Publisher hri_act_pub;

  public:
  T31(ros::NodeHandle node);
  void run();
};

