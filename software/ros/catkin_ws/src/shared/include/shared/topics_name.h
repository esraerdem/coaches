// defines the names of the topics to send data between modules
// #include "shared/topics_name.h"
#ifndef __TOPICS_NAME__
#define __TOPICS_NAME__

// general
#define TOPIC_ROBOT_LOCATION "amcl_pose"
#define TOPIC_MOVE_BASE "move_base"

// T11
#define TOPIC_KB "t11_knowledge"
#define SERVICE_GET_LOCATION "get_location"
#define SERVICE_GET_ALL_SITES "get_all_sites"

// T41
#define TOPIC_POLICY_RESULT "t41_policy_result"
#define TOPIC_PLANTOEXEC "planToExec"
#define TOPIC_PNPACTIVEPLACES "pnp/currentActivePlaces"
#define TOPIC_PNPCONDITION "PNPConditionEvent"

// T42
#define TOPIC_POLICY "t42_policy"


#endif
