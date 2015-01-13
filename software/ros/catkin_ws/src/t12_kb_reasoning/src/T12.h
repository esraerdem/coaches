/**
 * Integrates knowledge and events into the database
 * Receives knowledge from t11, environment events from t22 and human needs events from t32
 * Sends goals
 */

#include "ros/ros.h"
#include "t11_kb_modeling/Knowledge.h"
#include "shared/Event.h"
#include "shared/AllGoals.h"

#include <sstream>
#include <istream>

class T12 {
  private:
  ros::Publisher goals_pub;
  ros::Subscriber env_events_sub;
  ros::Subscriber human_needs_sub;
  ros::Subscriber knowledge_sub;

  void envEventsCallback(const shared::Event::ConstPtr& msg);
  void humanNeedsCallback(const shared::Event::ConstPtr& msg);
  void knowledgeCallback(const t11_kb_modeling::Knowledge::ConstPtr& msg);

  public:
  T12(ros::NodeHandle node);
  void run();
};

