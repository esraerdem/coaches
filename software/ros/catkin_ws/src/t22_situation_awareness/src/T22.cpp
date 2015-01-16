#include "T22.h"

T22::T22(ros::NodeHandle node) {
  event_pub = node.advertise<shared::Event>("t22_events", 100);
  feature_sub = node.subscribe("t21_features", 10, &T22::featureCallback, this);
}

void T22::featureCallback(const shared::Feature::ConstPtr& msg)
{
// TODO
}

void T22::run() {
  ros::spin();
}

