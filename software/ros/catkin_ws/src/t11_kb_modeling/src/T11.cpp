#include "T11.h"

T11::T11(ros::NodeHandle node) {
  knowledge_pub = node.advertise<t11_kb_modeling::Knowledge>("t11_knowledge", 100);
  position_sub = node.subscribe("t21_robot_location", 100, &T11::positionCallback, this);
  hri_feature_sub = node.subscribe("t31_feature", 10, &T11::hriFeatureCallback, this);
  env_feature_sub = node.subscribe("t21_feature", 10, &T11::envFeatureCallback, this);
  service_get_location = node.advertiseService("get_location", &T11::getLocation, this);
  service_get_all_sites = node.advertiseService("get_all_sites", &T11::getAllSites, this);
}

void T11::positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  // TODO
}

void T11::hriFeatureCallback(const shared::Feature::ConstPtr& msg) {
// TODO
}

void T11::envFeatureCallback(const shared::Feature::ConstPtr& msg) {
// TODO
}

bool T11::getAllSites(t11_kb_modeling::GetAllSites::Request  &req,
		      t11_kb_modeling::GetAllSites::Response &res) {
  //TODO
}

bool T11::getLocation(t11_kb_modeling::GetLocation::Request  &req,
		      t11_kb_modeling::GetLocation::Response &res) {
  //TODO
}

void T11::run() {
  ros::spin();
}

