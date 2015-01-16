#include "T42.h"

T42::T42(ros::NodeHandle node) {
  hri_goal_pub = node.advertise<shared::Goal>("t42_hri_goal", 100);
  nav_goal_pub = node.advertise<geometry_msgs::PoseStamped>("t42_nav_goal", 100);
  goal_set_sub = node.subscribe("t12_goals", 10, &T42::goalSetCallback, this);
}

void T42::goalSetCallback(const shared::AllGoals::ConstPtr& msg)
{
// TODO
}

void T42::run() {
  ros::spin();
}

