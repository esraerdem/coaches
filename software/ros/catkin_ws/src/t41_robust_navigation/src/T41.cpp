#include "T41.h"

T41::T41(ros::NodeHandle node) {
  low_level_pub = node.advertise<geometry_msgs::PoseStamped>("t41_low_level", 100);
  nav_goal_sub = node.subscribe("t42_nav_goal", 10, &T41::navGoalCallback, this);

  service_path_len = node.advertiseService("get_path_len", &T41::getPathLen, this);
}

void T41::navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
// TODO
  low_level_pub.publish(msg); // mock-up : sends to move_base_simple/goal
}

bool T41::getPathLen(t41_robust_navigation::GetPathLen::Request  &req,
		     t41_robust_navigation::GetPathLen::Response &res)
{
// TODO
  return false;
}

void T41::run() {
  ros::spin();
}

