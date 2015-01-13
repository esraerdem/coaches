/**
 * Converts raw features into knowledge
 * Receives features from HRI (t31) and sensors (T21)
 * Sends knowledge
 */

#include "ros/ros.h"
#include "shared/Feature.h"
#include "t11_kb_modeling/Knowledge.h"
#include <sstream>
#include <istream>

class T11 {
  private:
  ros::Publisher knowledge_pub;
  ros::Subscriber hri_feature_sub;
  ros::Subscriber env_feature_sub;
				   
  void hriFeatureCallback(const shared::Feature::ConstPtr& msg);
  void envFeatureCallback(const shared::Feature::ConstPtr& msg);

  public:
  T11(ros::NodeHandle node);
  void run();
};

