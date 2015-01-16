/**
 * Receives sets of goals from T12
 * Sends Naviagation goals to T41 and HRI goals to T31
 */

#include "ros/ros.h"
#include "shared/Goal.h"
#include "shared/AllGoals.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "t12_kb_reasoning/GetLocation.h"

#include <sstream>
#include <istream>

class T42 {
  private:
  ros::Publisher hri_goal_pub;
  ros::Publisher nav_goal_pub;
  ros::Subscriber goal_set_sub;
  ros::Subscriber location_sub;

  void goalSetCallback(const shared::AllGoals::ConstPtr& msg);
  void locationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  public:
  T42(ros::NodeHandle node);
  void run();
};

