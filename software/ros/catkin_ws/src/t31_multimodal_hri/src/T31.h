/**
 * Receives HRI Goals
 * Sends HRI features and actions
 */

#include "ros/ros.h"
#include "shared/Feature.h"
#include "shared/Goal.h"
#include "t31_multimodal_hri/HRIActuation.h"
#include <sstream>
#include <istream>

class T31 {
  private:
  ros::Subscriber hri_goal_sub;

  ros::Publisher feature_pub;
  ros::Publisher hri_act_pub;

  void hriGoalCallback(const shared::Goal::ConstPtr& msg);

  public:
  T31(ros::NodeHandle node);
  void run();
};

