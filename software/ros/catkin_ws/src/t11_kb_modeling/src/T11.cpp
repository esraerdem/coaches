#include "T11.h"

T11::T11(ros::NodeHandle node) {
  knowledge_pub = node.advertise<t11_kb_modeling::Knowledge>("t11_knowledge", 100);
  hri_feature_sub = node.subscribe("t31_features", 10, &T11::hriFeatureCallback, this);
  env_feature_sub = node.subscribe("t21_features", 10, &T11::envFeatureCallback, this);
}

void T11::hriFeatureCallback(const shared::Feature::ConstPtr& msg)
{
// TODO
}

void T11::envFeatureCallback(const shared::Feature::ConstPtr& msg)
{
// TODO
}

void T11::run() {
  ros::spin();
}

