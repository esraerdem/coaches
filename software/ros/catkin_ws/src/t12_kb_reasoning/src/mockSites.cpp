/**
 * Mock-up module that sends shop-sites with random values every minute
 * Implements the get_location service
 * Sends goals
 */

#include "ros/ros.h"
#include "shared/AllGoals.h"
#include "t12_kb_reasoning/GetLocation.h"
#include "geometry_msgs/Point.h"

#include <sstream>
#include <istream>
#include <map>

#define pmap std::map<std::string, geometry_msgs::Point>

class MockSites {
  private:
  ros::Publisher goals_pub;
  ros::ServiceServer service_get_location;
  pmap locations;

  bool getLocation(t12_kb_reasoning::GetLocation::Request  &req,
		   t12_kb_reasoning::GetLocation::Response &res);

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

  service_get_location = node.advertiseService("get_location", &MockSites::getLocation, this);

  locations["doorWest"]=mkPoint(6,19);
  locations["doorEast"]=mkPoint(93.69,29.75);
  locations["monoprix"]=mkPoint(78,24);
  locations["phone"]=mkPoint(89,31);
  locations["restaurant"]=mkPoint(10,23);
  locations["carPark"]=mkPoint(27,20);
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
  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mockSites");

  ros::NodeHandle node;

  MockSites mock(node);
  mock.run();

  return 0;
}

