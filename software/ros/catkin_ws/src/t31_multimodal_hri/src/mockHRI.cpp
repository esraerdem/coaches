/**
 * Receives Laser scans and camera images.
 * Sends features and robot location.
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

using namespace std;

class MockHRI {
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
  MockHRI(ros::NodeHandle node);
  void run();
};

MockHRI::MockHRI(ros::NodeHandle node) {
  hri_goal_sub = node.subscribe(TOPIC_HRI_GOAL, 10, &MockHRI::hriGoalCallback, this);
  location_sub = node.subscribe(TOPIC_ROBOT_LOCATION, 10, &MockHRI::locationCallback, this);
  tcp_sub = node.subscribe(TOPIC_RCOMMESSAGE, 10, &MockHRI::tcpCallback, this);

  feature_pub = node.advertise<shared::Feature>("t31_feature", 100);
  hri_act_pub = node.advertise<t31_multimodal_hri::HRIActuation>("hri_actuation", 100);
  say_stage_pub = node.advertise<std_msgs::String>(TOPIC_STAGE_SAY, 100);
  PNP_cond_pub = node.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 100);

  interactionTimer = node.createTimer(ros::Duration(5), &MockHRI::intTimerCallback, this, true); interactionTimer.stop();
  adTimer = node.createTimer(ros::Duration(3), &MockHRI::adTimerCallback, this, true); adTimer.stop();
  currentInteraction = "";
}

void MockHRI::locationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  robot = msg->pose.pose;
}

void MockHRI::tcpCallback(tcp_interface::RCOMMessage msg) {
    string sm = msg.value;
    if (sm!="") {
        boost::algorithm::to_lower(sm);
        vector<string> toks;
        boost::split(toks,sm,boost::is_any_of("()\" \n\r"));
        for (vector<string>::iterator it = toks.begin(); it != toks.end(); ++it) {
            std_msgs::String out;
            if (*it=="yes" || *it=="no" || 
                *it=="schedule" || *it=="toilet" || *it=="adminroom") {
                out.data = *it;
                PNP_cond_pub.publish(out);
                cout << "Published PNP condition from ASR: " << out.data << endl;
            }
            
        }
    }
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
  } else if (msg->kind == GOAL_SPEECH) {
    msgOut.data = msg->param;
    interactionTimer.setPeriod(ros::Duration(3+msgOut.data.length()/10.));
    interactionTimer.start();
  }
  currentInteraction = msg->param;
  doSay(msgOut);
}

void MockHRI::doSay(std_msgs::String msg) {
  if (msg.data!="") {
    std::cout << "Say \033[22;35;1m" << msg.data << "\033[0m" << std::endl;
    msg.data += "      ."; // if not clearing balloon, adding a few spaces to enlarge the balloon
  }
  say_stage_pub.publish(msg);  // received by stage to print out say message
  if (false && msg.data!="") {
      std::stringstream ss; ss << "pico2wave -w /tmp/say_out.wav \"" << msg.data << "\"";
      int r1=system(ss.str().c_str());
      int r2=system("aplay /tmp/say_out.wav");
      if (r1<0 || r2<0) {
	  std::cerr << "ERROR in Say procedure (pico2wave+aplay)" << std::endl;
      }
  }
  
  // Possibly add functions to also display what the robot says
  
}
 
 
void MockHRI::doDisplay(std_msgs::String msg) {
  if (msg.data!="") {
    std::cout << "Display \033[22;35;1m" << msg.data << "\033[0m" << std::endl;
    msg.data += "      ."; // if not clearing balloon, adding a few spaces to enlarge the balloon
  }
  say_stage_pub.publish(msg);  // received by stage to print out say message
  if (false && msg.data!="") {
      std::stringstream ss; ss << "pico2wave -w /tmp/say_out.wav \"" << msg.data << "\"";
      int r1=system(ss.str().c_str());
      int r2=system("aplay /tmp/say_out.wav");
      if (r1<0 || r2<0) {
          std::cerr << "ERROR in Say procedure (pico2wave+aplay)" << std::endl;
      }
  }
  
  // Use tcp_interface to send 'display_{text|image|video}_{interactionname}'
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
  doSay(msgOut);   // to delete the say message in stage
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
  doSay(msgOut);  // to delete the say message in stage
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

