#include "T12.h"

T12::T12(ros::NodeHandle node) {
  goals_pub = node.advertise<shared::AllGoals>("t12_goals_set", 100);
  env_events_sub = node.subscribe("t22_event", 10, &T12::envEventsCallback, this);
  human_needs_sub = node.subscribe("t32_event", 10, &T12::humanNeedsCallback, this);
  knowledge_sub = node.subscribe("t11_knowledge", 10, &T12::knowledgeCallback, this);

  service_get_location = node.advertiseService("get_location", &T12::getLocation, this);
}

void T12::envEventsCallback(const shared::Event::ConstPtr& msg)
{
// TODO
}

void T12::humanNeedsCallback(const shared::Event::ConstPtr& msg)
{
// TODO
}

void T12::knowledgeCallback(const t11_kb_modeling::Knowledge::ConstPtr& msg)
{
// TODO
}

bool T12::getLocation(t12_kb_reasoning::GetLocation::Request  &req,
		      t12_kb_reasoning::GetLocation::Response &res)
{
// TODO
  return false;
}

void T12::run() {
  ros::spin();
}

