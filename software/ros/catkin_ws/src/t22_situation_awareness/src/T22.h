/**
 * Receives Features from T21.
 * Sends Events.
 */

#include "ros/ros.h"
#include "shared/Feature.h"
#include "shared/Event.h"

#include <sstream>
#include <istream>

class T22 {
  private:
  ros::Publisher event_pub;
  ros::Subscriber feature_sub;

  void featureCallback(const shared::Feature::ConstPtr& msg);

  public:
  T22(ros::NodeHandle node);
  void run();
};

