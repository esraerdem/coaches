#include "T31.h"

T31::T31(ros::NodeHandle node) {
  feature_pub = node.advertise<shared::Feature>("t31_features", 100);
  hri_act_pub = node.advertise<t31_multimodal_hri::HRIActuation>("hri_actuations", 100);
}

void T31::run() {
  ros::spin();
}

