/**
 * Mock-up module that sends shop-sites with random values every minute
 * Implements the get_location service
 * Sends goals
 */

#include "ros/ros.h"
#include "shared/AllGoals.h"
#include "t12_kb_reasoning/GetLocation.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <sstream>
#include <istream>
#include <map>

#define RATE (1/15.)

typedef std::map<std::string, geometry_msgs::Point> pmap;
typedef std::map<std::string, float> cmap;

class MockSites {
  private:
  ros::Publisher goals_pub;
  ros::Subscriber position_sub;
  ros::ServiceServer service_get_location;
  pmap locations;
  cmap visits;

  void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  bool getLocation(t12_kb_reasoning::GetLocation::Request  &req,
		   t12_kb_reasoning::GetLocation::Response &res);

  void sendGoals();

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
  goals_pub = node.advertise<shared::AllGoals>("t12_goals_set", 100);

  position_sub = node.subscribe("t21_robot_location", 100, &MockSites::positionCallback, this);

  service_get_location = node.advertiseService("get_location", &MockSites::getLocation, this);

  locations["doorWest"]=mkPoint(6,19);
  locations["doorEast"]=mkPoint(93.69,29.75);
  locations["monoprix"]=mkPoint(78,24);
  locations["phone"]=mkPoint(89,31);
  locations["restaurant"]=mkPoint(10,23);
  locations["carPark"]=mkPoint(27,20);

  pmap::iterator it = locations.begin(); 
  while (it != locations.end()) {
    visits[it->first] = 0;
    ++it;
  }
}

static inline double sq(double x) {return x*x;}

void MockSites::positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  geometry_msgs::Point robot = msg->pose.pose.position;
  ROS_INFO("Robot at %f %f",robot.x, robot.y);
  pmap::iterator it = locations.begin(); 
  while (it != locations.end()) {
    if (sq(robot.x - it->second.x)+sq(robot.y - it->second.y) < 1) {
      visits[it->first] = 0;
      ROS_INFO("Visited site %s",it->first.c_str());
    } /*else
	ROS_INFO("Site %s distant of %f",it->first.c_str(), sq(robot.x - it->second.y)+sq(robot.y - it->second.y) );*/
    ++it;
  }
}

bool MockSites::getLocation(t12_kb_reasoning::GetLocation::Request  &req,
		      t12_kb_reasoning::GetLocation::Response &res)
{
  std::cout << "Requesting " << req.loc << std::endl;
  std::string id(req.loc);
  pmap::iterator it = locations.find(id);
  if (it != locations.end()) {
    res.coords.position = it->second;
    res.coords.orientation.x = 0;
    res.coords.orientation.y = 0;
    res.coords.orientation.z = 0;
    res.coords.orientation.w = 1;
    return true;
  } else {
    ROS_WARN("location %s is unknown",id.c_str());
    return false;
  }
}

void MockSites::run() {
  ros::Rate loop_rate(RATE);
  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
    sendGoals();
  }
}

void MockSites::sendGoals() {
  shared::Goal g;
  shared::AllGoals all;
  cmap::iterator it = visits.begin();
  all.mode = "patrol";
  while (it != visits.end()) {
    g.loc = it->first;
    g.kind = "look";
    g.value = it->second;
    all.goals.push_back(g);
    it->second ++;
    ++it;
  }
  all.header.stamp.sec = ros::Time::now().sec;
  all.header.stamp.nsec = ros::Time::now().nsec;
  goals_pub.publish(all);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mockSites");

  ros::NodeHandle node;

  MockSites mock(node);
  mock.run();

  return 0;
}

