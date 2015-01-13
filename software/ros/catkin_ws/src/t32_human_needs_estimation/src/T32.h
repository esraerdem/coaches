/**
 * Estimates human needs
 * Receives Features from T31.
 * Sends Events.
 */

#include "ros/ros.h"
#include "shared/Feature.h"
#include "shared/Event.h"

#include <sstream>
#include <istream>

class T32 {
  private:
  ros::Publisher event_pub;
  ros::Subscriber hri_feature_sub;
  ros::Subscriber people_feature_sub;

  void hriFeatureCallback(const shared::Feature::ConstPtr& msg);
  void peopleFeatureCallback(const shared::Feature::ConstPtr& msg);

  public:
  T32(ros::NodeHandle node);
  void run();
};

