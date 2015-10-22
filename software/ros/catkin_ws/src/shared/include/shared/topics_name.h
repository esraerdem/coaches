// defines the names of the topics to send data between modules
// #include "shared/topics_name.h"
#ifndef __TOPICS_NAME__
#define __TOPICS_NAME__

// general
#define TOPIC_ROBOT_LOCATION "amcl_pose"
#define TOPIC_MOVE_BASE "move_base"
#define TOPIC_RCOMMESSAGE "/RCOMMessage"

// T11
#define TOPIC_KB "t11_knowledge"
#define SERVICE_GET_LOCATION "get_location"
#define SERVICE_GET_ALL_SITES "get_all_sites"

// T31
#define TOPIC_SAY "say"
#define TOPIC_STAGE_SAY "stage_say"
#define TOPIC_HRI_GOAL "t31_hri_goal"
#define TOPIC_LASER_OBSMAP "laser_obstacle_map"


// T41
#define TOPIC_POLICY_RESULT "t41_policy_result"
#define TOPIC_PLANTOEXEC "planToExec"
#define TOPIC_PNPACTIVEPLACES "pnp/currentActivePlaces"
#define TOPIC_PNPCONDITION "PNPConditionEvent"

// T42
#define TOPIC_POLICY "t42_policy"


#endif
