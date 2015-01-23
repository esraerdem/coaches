#include "T41.h"

T41::T41(ros::NodeHandle node) {
  this->node = node;
  low_level_pub = node.advertise<geometry_msgs::PoseStamped>("t41_low_level", 100);
  nav_goal_sub = node.subscribe("t42_nav_goal", 10, &T41::navGoalCallback, this);

  service_path_len = node.advertiseService("get_path_len", &T41::getPathLen, this);
}

void T41::navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
// TODO
  low_level_pub.publish(msg); // mock-up : sends to move_base_simple/goal
}

static inline double sq(double v) { return v*v; };

bool T41::getPathLen(t41_robust_navigation::GetPathLen::Request  &req,
		     t41_robust_navigation::GetPathLen::Response &res)
{
  nav_msgs::GetPlan srv;
  srv.request.start = req.from;
  srv.request.start.header.frame_id="map";
  srv.request.goal = req.to;
  srv.request.goal.header.frame_id="map";
  srv.request.tolerance = 2.1;
  ros::ServiceClient planner = node.serviceClient<nav_msgs::GetPlan>("/diago/move_base_node/make_plan");
  if (planner.call(srv)) {
    // Now computes the path length
    double len = 0;
    geometry_msgs::Point prev;
    std::vector<geometry_msgs::PoseStamped>::iterator it = srv.response.plan.poses.begin();
    if (it != srv.response.plan.poses.end()) {
      prev = it->pose.position;
      ++it;
    }
    while (it != srv.response.plan.poses.end()) {
      geometry_msgs::Point cur = it->pose.position;
      len += sqrt(sq(cur.x-prev.x)+sq(cur.y-prev.y));
      ++it;
      prev = cur;
    }
    res.length = len;
    return true;
  } else {
    ROS_WARN("Unable to compute Path");
    return false;
  }
}

void T41::run() {
  ros::spin();
}

