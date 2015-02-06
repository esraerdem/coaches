/**
 * Receives /people data from the simulator.
 * Sends people features
 */

#include "ros/ros.h"
#include "shared/Feature.h"
#include "shared/featureKind.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"

#include <sstream>
#include <istream>

class MockPeople {
  private:
  ros::Publisher feature_pub;
  ros::Subscriber people_sub;
  std::list<std::string> mylist; // list of people id for uid tracking

  void peopleCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  public:
  MockPeople(ros::NodeHandle node);
  void run();
  int32_t getUID(std::string id); // Gets the uid from the people id (creates one if necessary)
};

MockPeople::MockPeople(ros::NodeHandle node) {
  feature_pub = node.advertise<shared::Feature>("t21_feature", 100);
  people_sub = node.subscribe("/people", 100, &MockPeople::peopleCallback, this);
}

int32_t MockPeople::getUID (std::string id) {  
  int32_t idx=0;
  for (std::list<std::string>::iterator it=mylist.begin(); it != mylist.end(); ++it)
    if (*it == id)
      return idx;
    else
      idx++;
  mylist.push_back(id);
  return idx;
}

void MockPeople::peopleCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  shared::Feature data;
  data.header = msg->header;
  data.header.frame_id = "/map";
  data.kind = DETECT_PEOPLE;
  data.location = msg->pose;
  /*
  data.location.x = msg->pose.position.x;
  data.location.y = msg->pose.position.y;
  double q0 = msg->pose.orientation.x;
  double q1 = msg->pose.orientation.y;
  double q2 = msg->pose.orientation.z;
  double q3 = msg->pose.orientation.w;
  //  double yaw = atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3));
  double yaw = atan2(2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2));
  data.location.theta = yaw;
  */
  std::stringstream converter;
  converter << getUID(std::string(msg->header.frame_id));
  data.uid = converter.str();
  feature_pub.publish(data);
}

void MockPeople::run() {
  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mock_people");

  ros::NodeHandle node;

  MockPeople mock(node);
  mock.run();

  return 0;
}

