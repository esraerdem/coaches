/**
 * Receives HRI Goals
 * Sends HRI features and actions
 */


#include "ros/ros.h"
#include "shared/Feature.h"
#include "shared/Goal.h"
#include "shared/goalKind.h"
#include "shared/featureKind.h"
#include "shared/topics_name.h"
#include "t31_multimodal_hri/HRIActuation.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include <tcp_interface/RCOMMessage.h>

#include <boost/algorithm/string.hpp>

#include <sstream>
#include <istream>

class T31 {
  private:
  ros::Subscriber hri_goal_sub;
  ros::Subscriber location_sub;
  ros::Subscriber tcp_sub; // receiving data from tcp_interface

  ros::Publisher feature_pub;
  ros::Publisher hri_act_pub;
  ros::Publisher say_stage_pub;
  ros::Publisher PNP_cond_pub;
  ros::Timer interactionTimer;
  ros::Timer adTimer;

  geometry_msgs::Pose robot;
  std::string currentInteraction;

  void locationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void hriGoalCallback(const shared::Goal::ConstPtr& msg);
  void tcpCallback(tcp_interface::RCOMMessage msg);
  void doSay(std_msgs::String msg);
  void doDisplay(std_msgs::String msg);
  
  void intTimerCallback(const ros::TimerEvent&);
  void adTimerCallback(const ros::TimerEvent&);

  public:
  T31(ros::NodeHandle node);
  void run();
};

