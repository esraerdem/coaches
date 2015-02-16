/**
 * Receives nav goals from T42
 * Sends low-level commands to the robot
 */

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetPlan.h"
#include <nav_msgs/Odometry.h>
#include "t41_robust_navigation/GetPathLen.h"
#include "t41_robust_navigation/Policy.h"

#include <sstream>
#include <istream>

class T41 {
  private:
  ros::NodeHandle node;
  ros::Publisher low_level_pub, plantoexec_pub;
  ros::Subscriber nav_goal_sub, location_sub, policy_sub;
  ros::ServiceServer service_path_len;
  geometry_msgs::Pose robot_pose;
  std::string robotname, planfolder;

  void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  // void locationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void amclposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void policyCallback(const t41_robust_navigation::Policy::ConstPtr& msg);

  bool getPathLen(t41_robust_navigation::GetPathLen::Request  &req,
		  t41_robust_navigation::GetPathLen::Response &res);

  public:
  T41(ros::NodeHandle node);
  void run();
};

