/**
 * Mock-up module that sends shop-sites with patrol values every 15s
 * Receives people events and builds goals from them
 * Sends goals
 */

#include "ros/ros.h"
#include "shared/Feature.h"
#include "shared/Event.h"
#include "shared/AllGoals.h"
#include "shared/goalKind.h"
#include "shared/featureKind.h"
#include "shared/eventKind.h"
#include "shared/topics_name.h"
#include "t11_kb_modeling/GetAllSites.h"
#include "t11_kb_modeling/GetLocation.h"
#include "t11_kb_modeling/Knowledge.h"
#include "t11_kb_modeling/knowledgeKind.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "t41_robust_navigation/policyResults.h"
#include "t41_robust_navigation/PolicyResult.h"

#include <sstream>
#include <istream>
#include <map>
#include <boost/algorithm/string.hpp>

#define PATROL_RATE (1/15.)

enum State { detected, request, escort, follow, escort2 };

struct people {
  enum State currentState;
  std::string param;
};

typedef struct people People;
typedef std::map<std::string, float> cmap;
typedef std::map<std::string, People> pmap;

class MockSites {
  private:
  ros::NodeHandle node;
  ros::Publisher goals_pub;
  ros::Subscriber knowledge_sub, need_sub, event_sub, result_sub;

  ros::Timer timPatrol;
  cmap visits;
  pmap peoples;
  std::string onSite;

  void knowledgeCallback(const t11_kb_modeling::Knowledge::ConstPtr& msg);
  void needCallback(const shared::Event::ConstPtr& msg);
  void eventCallback(const shared::Event::ConstPtr& msg);
  void resultCallback(const t41_robust_navigation::PolicyResult::ConstPtr& msg);

  void sendPatrols(const ros::TimerEvent&);
  void sendInteractGoals();

  public:
  MockSites(ros::NodeHandle node);
  void run();
};

static inline geometry_msgs::Point mkPoint(float x,float y) {
  geometry_msgs::Point res;
  res.x = x;
  res.y = y;
  res.z = 0;
  return res;
}
MockSites::MockSites(ros::NodeHandle node) {
  this->node = node;
  ros::ServiceClient service_get_all_sites = node.serviceClient<t11_kb_modeling::GetAllSites>("get_all_sites");
  t11_kb_modeling::GetAllSites srv;
  if (! service_get_all_sites.call(srv)) {
    ROS_WARN("Unable to get sites");
    exit(1);
  }
  std::vector<std::string>::const_iterator it = srv.response.ids.begin();
  while (it != srv.response.ids.end()) {
    visits[*it] = 500;
    ++it;
  }

  goals_pub = node.advertise<shared::AllGoals>("t12_goals_set", 100);

  knowledge_sub = node.subscribe("t11_knowledge", 100, &MockSites::knowledgeCallback, this);
  need_sub = node.subscribe("t32_event", 100, &MockSites::needCallback, this);
  event_sub = node.subscribe("t22_event", 100, &MockSites::eventCallback, this);
  result_sub = node.subscribe("t41_policy_result", 10, &MockSites::resultCallback, this);
  
  onSite = "";
}

static inline double sq(double x) {return x*x;}

void MockSites::knowledgeCallback(const t11_kb_modeling::Knowledge::ConstPtr& msg) {
  if (msg->category==KB_VISIT) {
    cmap::iterator it = visits.begin(); 
    std::string old = onSite;
    onSite = "";
    while (it != visits.end()) {
      if (it->first == msg->knowledge) {
	it->second = 0;
	onSite = it->first;
	if (old != onSite) {
          old = onSite;
          ros::TimerEvent dummy;
          sendPatrols(dummy);
        }
      } else {
	++ it->second;
      }
      ++it;
    }
  } else if (msg->category==KB_ROBOT_AT) {
    cmap::iterator it = visits.begin(); 
    onSite = "";
    while (it != visits.end()) {
      ++ it->second;
      ++it;
    }
  } else if (msg->category==KB_PEOPLE) {
    peoples[msg->knowledge].currentState = detected;
  }
}

void MockSites::needCallback(const shared::Event::ConstPtr& msg) {
  if (msg->kind == INTERACTION_COMPLETE) {
    // TODO
    switch (peoples[msg->uid].currentState) {
    case request:
      peoples[msg->uid].currentState = detected; 
      return; // interaction complete, don't generate a new goal
    case escort:
      peoples[msg->uid].currentState = follow; break;
    case follow:
      peoples[msg->uid].currentState = escort2; break;
    case escort2:
      peoples[msg->uid].currentState = detected; break;
    }
    sendInteractGoals();
  } else if (msg->kind == NEED_ESCORT) {
    peoples[msg->uid].currentState = escort;
    peoples[msg->uid].param = msg->param;
    sendInteractGoals();
  }
}

void MockSites::eventCallback(const shared::Event::ConstPtr& msg) {
  if (msg->kind == EVENT_REQUEST) {
    peoples[msg->uid].currentState = request;
    sendInteractGoals();
  }
}

void MockSites::resultCallback(const t41_robust_navigation::PolicyResult::ConstPtr& msg) {
  // some action has completed or failed
  if (msg->feedback == POLICY_SUCCESS) {
    // that was success
    // let's remove the corresponding reward and replan
    std::vector<std::string> tokens;
    boost::split( tokens, msg->feedback, boost::is_any_of("( )"), boost::token_compress_on );
    // here tokens should contain 'action','location' [, parameters]

  } // if policy has succeeded
}

// r t21_robot_location:=/diago/amcl_pose
void MockSites::run() {
  timPatrol = node.createTimer(ros::Duration(1.0/PATROL_RATE), &MockSites::sendPatrols, this, true);
  ros::spin();
}

void MockSites::sendInteractGoals() {
  shared::Goal g;
  shared::AllGoals all;
  all.mode = "interact";
  pmap::iterator it = peoples.begin();
  while (it != peoples.end()) {
    switch (it->second.currentState) {
    case detected: break;
    case request:
      g.kind = GOAL_INTERACT;
      g.loc = "_P_"+it->first;
      g.param = it->first;
      g.value = 1000;
      g.duration = 60;
      all.goals.push_back(g);
      break;
    case escort:
      g.kind = GOAL_ESCORT;
      g.loc = it->second.param;
      g.param = it->first;
      g.value = 2000;
      g.duration = 300;
      all.goals.push_back(g);
      break;
    case follow:
      g.kind = GOAL_FOLLOW;
      g.loc = "_P_"+it->first;
      g.param = it->first;
      g.value = 20000;
      g.duration = 3;
      all.goals.push_back(g);
      break;
    case escort2:
      g.kind = GOAL_ESCORT2;
      g.loc = it->second.param;
      g.param = it->first;
      g.value = 2000;
      g.duration = 300;
      all.goals.push_back(g);
      break;
    }
    ++it;
  } // while *it in peoples
  all.header.stamp = ros::Time::now();
  goals_pub.publish(all);
  ROS_DEBUG("send interact goals");  
} // sendInteractGoals()

void MockSites::sendPatrols(const ros::TimerEvent&) {
  if (ros::Time::now().sec<1) return;
  shared::Goal g;
  shared::AllGoals all;
  cmap::iterator it = visits.begin();
  all.mode = "advertise";
  while (it != visits.end()) {
    g.loc = it->first;
    if (rand()<RAND_MAX/4) {
      g.kind = GOAL_ADVERTISE_COMPLEX;
      g.param = GOAL_SWIPE;
      g.value = it->second / 100;
      g.duration = 5;
    } else {
      g.kind = GOAL_ADVERTISE;
      g.param = "ad";
      g.value = it->second / 100;
      g.duration = 3;
    }
    all.goals.push_back(g);
    /*
    if (it->first != onSite)
      it->second ++; // prevent increasing the reward where the robot is
     */
    ++it;
  }
  all.header.stamp.sec = ros::Time::now().sec;
  all.header.stamp.nsec = ros::Time::now().nsec;
  goals_pub.publish(all);
  ROS_DEBUG("send patrol goals");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mockSites");

  ros::NodeHandle node;

  MockSites mock(node);
  mock.run();

  return 0;
}

