#include "T32.h"

T32::T32(ros::NodeHandle node) {
  event_pub = node.advertise<shared::Event>("t32_event", 100);
  hri_feature_sub = node.subscribe("t31_feature", 10, &T32::hriFeatureCallback, this);
  people_feature_sub = node.subscribe("t21_feature", 10, &T32::peopleFeatureCallback, this);
}

void T32::hriFeatureCallback(const shared::Feature::ConstPtr& msg)
{
// TODO
}

void T32::peopleFeatureCallback(const shared::Feature::ConstPtr& msg)
{
// TODO
}

void T32::run() {
  ros::spin();
}

