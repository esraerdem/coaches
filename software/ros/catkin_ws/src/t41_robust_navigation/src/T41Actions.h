#ifndef _MY_ACTIONS_
#define _MY_ACTIONS_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>

#include <map>
#include <boost/thread/thread.hpp>


using namespace std;

// Action utilities

bool getRobotPose(string robotname, double &x, double &y, double &th_rad);
bool getLocationPosition(string loc, double &GX, double &GY);
void do_movebase(string robotname, float GX, float GY, float GTh_DEG, bool *run); // theta in degrees

#endif

