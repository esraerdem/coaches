/**
 * Receives Laser scans and camera images.
 * Sends features and robot location.
 */

#include "ros/ros.h"
#include "shared/Feature.h"
#include "shared/Goal.h"
#include "shared/goalKind.h"
#include "shared/featureKind.h"
#include "t31_multimodal_hri/HRIActuation.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"

#include <sstream>
#include <istream>

class MockHRI {
  private:
  ros::Subscriber hri_goal_sub;
  ros::Subscriber location_sub;

  ros::Publisher feature_pub;
  ros::Publisher hri_act_pub;
  ros::Publisher say_pub;
  ros::Timer interactionTimer;
  ros::Timer adTimer;

  geometry_msgs::Pose robot;
  std::string currentInteraction;

  void locationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void hriGoalCallback(const shared::Goal::ConstPtr& msg);
  
  void intTimerCallback(const ros::TimerEvent&);
  void adTimerCallback(const ros::TimerEvent&);

  public:
  MockHRI(ros::NodeHandle node);
  void run();
};

MockHRI::MockHRI(ros::NodeHandle node) {
  hri_goal_sub = node.subscribe("t42_hri_goal", 10, &MockHRI::hriGoalCallback, this);
  location_sub = node.subscribe("t21_robot_location", 10, &MockHRI::locationCallback, this);

  feature_pub = node.advertise<shared::Feature>("t31_feature", 100);
  hri_act_pub = node.advertise<t31_multimodal_hri::HRIActuation>("hri_actuation", 100);
  say_pub = node.advertise<std_msgs::String>("say", 100);
  interactionTimer = node.createTimer(ros::Duration(5), &MockHRI::intTimerCallback, this, true); interactionTimer.stop();
  adTimer = node.createTimer(ros::Duration(3), &MockHRI::adTimerCallback, this, true); adTimer.stop();
  currentInteraction = "";
}

void MockHRI::locationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  robot = msg->pose.pose;
}

void MockHRI::hriGoalCallback(const shared::Goal::ConstPtr& msg)
{
  std_msgs::String msgOut;
  if (msg->kind == GOAL_ADVERTISE) {
    msgOut.data = "Advertisement: " + msg->param;
    adTimer.setPeriod(ros::Duration(3));
    adTimer.start();
  } else if (msg->kind == GOAL_INTERACT) {
    msgOut.data = "Interacting with " + msg->param;
    interactionTimer.setPeriod(ros::Duration(5));
    interactionTimer.start();
  } else if (msg->kind == GOAL_FOLLOW) {
    msgOut.data = "Follow me to " + msg->loc;
    interactionTimer.setPeriod(ros::Duration(2));
    interactionTimer.start();
  } else if (msg->kind == GOAL_DONE) {
    msgOut.data = "We arrived to " + msg->loc + ", have a nice day!";
    interactionTimer.setPeriod(ros::Duration(3));
    interactionTimer.start();
  }
  currentInteraction = msg->param;
  say_pub.publish(msgOut);
}

void MockHRI::intTimerCallback(const ros::TimerEvent&) {
  shared::Feature msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id="diago";
  msg.kind = HRI_DONE;
  msg.location = robot;
  msg.uid = currentInteraction;
  feature_pub.publish(msg);
  std_msgs::String msgOut;
  msgOut.data = "";
  say_pub.publish(msgOut);
  currentInteraction = "";
}

void MockHRI::adTimerCallback(const ros::TimerEvent&) {
  shared::Feature msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id="diago";
  msg.kind = HRI_AD;
  msg.location = robot;
  msg.uid = currentInteraction;
  feature_pub.publish(msg);
  std_msgs::String msgOut;
  msgOut.data = "";
  say_pub.publish(msgOut);
  currentInteraction = "";
}


void MockHRI::run() {
  ros::spin();
}

/**
 * Deals with Human Robot Interaction.
 * Sends features and HRI actuations
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mockHRI");

  ros::NodeHandle node;

  MockHRI mock(node);
  mock.run();

  return 0;
}

