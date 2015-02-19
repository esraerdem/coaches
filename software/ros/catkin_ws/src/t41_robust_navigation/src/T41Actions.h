#ifndef _MY_ACTIONS_
#define _MY_ACTIONS_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

//#include <rococo_navigation/FollowCorridorAction.h>
//#include <rococo_navigation/FollowPersonAction.h>
//#include <rp_action_msgs/TurnAction.h>
//#include <coaches_reasoning/AskQuestionAction.h>
//#include "semantic_utils.h"
#include <map>
#include <boost/thread/thread.hpp>
#include <std_msgs/String.h>
//#include <rococo_navigation/TurnAction.h>
//#include <coaches_msgs/PedestrianInfo.h>



using namespace std;

// Action implementation

void advertise(string params, bool *run);
void interact(string params, bool *run);
void fake(string params, bool *run);

#endif

