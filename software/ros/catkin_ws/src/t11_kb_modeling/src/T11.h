/**
 * Converts raw features into knowledge
 * Receives features from HRI (t31) and sensors (T21)
 * Sends knowledge
 */

#include "ros/ros.h"
#include "shared/Feature.h"
#include "shared/featureKind.h"
#include "t11_kb_modeling/GetLocation.h"
#include "t11_kb_modeling/GetAllSites.h"
#include "t11_kb_modeling/Knowledge.h"
#include "t11_kb_modeling/knowledgeKind.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <sstream>
#include <istream>

class T11 {
  private:
  ros::Publisher knowledge_pub;
  ros::Subscriber position_sub, hri_feature_sub, env_feature_sub;
  ros::ServiceServer service_get_location, service_get_all_sites;

  bool getLocation(t11_kb_modeling::GetLocation::Request  &req,
		   t11_kb_modeling::GetLocation::Response &res);

  bool getAllSites(t11_kb_modeling::GetAllSites::Request  &req,
		   t11_kb_modeling::GetAllSites::Response &res);

				   
  void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void hriFeatureCallback(const shared::Feature::ConstPtr& msg);
  void envFeatureCallback(const shared::Feature::ConstPtr& msg);

  public:
  T11(ros::NodeHandle node);
  void run();
};

