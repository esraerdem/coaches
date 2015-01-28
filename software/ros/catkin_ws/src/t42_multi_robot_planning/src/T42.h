/**
 * Receives sets of goals from T12
 * Sends Naviagation goals to T41 and HRI goals to T31
 */

#include "ros/ros.h"
#include "shared/Goal.h"
#include "shared/AllGoals.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "t41_robust_navigation/GetPathLen.h"
#include "t12_kb_reasoning/GetLocation.h"

#include <sstream>
#include <istream>
#include <map>
#include <vector>

// r t21_robot_location:=/diago/amcl_pose get_path_len:=/diago/get_path_len __name:=multi_robot_planning __ns:=diago

typedef geometry_msgs::Point Point;

class T42 {
  private:
  ros::NodeHandle node;
  ros::Publisher hri_goal_pub;
  ros::Publisher nav_goal_pub;
  ros::Subscriber goal_set_sub;
  ros::Subscriber location_sub;

  Point robot;
  bool positionUpdated;
  ros::Timer positionTimer;

  std::map<std::string, int> fixedSites;
  std::vector<Point> sitesLocation;
  std::vector<std::vector<float> > site2siteDistance; // distances from site to site : a vector for each fixed site
  std::vector<float> robot2siteDistance; // distances from the robot to each fixed site
  std::vector<std::map<std::string,float> > siteActionReward; // a map action->reward for each fixed site

  int newFixedSite(std::string site);
  void positionTimerCallback(const ros::TimerEvent&);

  void goalSetCallback(const shared::AllGoals::ConstPtr& msg);
  void locationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  public:
  T42(ros::NodeHandle node);
  void run();
};

