#include <boost/thread/thread.hpp>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <boost/algorithm/string.hpp>
#include <shared/topics_name.h>
#include <shared/Goal.h>
#include <shared/goalKind.h>

#include "T41PNPAS.h"
#include "T41Actions.h"

#define DEG(a) ((a)*180.0/M_PI)


T41PNPActionServer::T41PNPActionServer() : PNPActionServer() {
    event_pub = handle.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 10);
    plantoexec_pub = handle.advertise<std_msgs::String>(TOPIC_PLANTOEXEC, 100);
    hri_pub = handle.advertise<shared::Goal>(TOPIC_HRI_GOAL, 10);


    //position_sub = handle.subscribe("odom", 10, &MyPNPActionServer::odom_callback, this);

    register_action("advertise",&T41PNPActionServer::advertise,this);
    register_action("advertiseComplex",&T41PNPActionServer::advertiseComplex,this);
    register_action("swipe",&T41PNPActionServer::swipe,this);
    register_action("interact",&T41PNPActionServer::interact,this);
    register_action("wait",&T41PNPActionServer::wait,this);
    register_action("turn",&T41PNPActionServer::turn,this);
    register_action("goto",&T41PNPActionServer::followcorridor,this);
    register_action("say",&T41PNPActionServer::say,this);
    register_action("restart",&T41PNPActionServer::restart,this);

    handle.param("robot_name",robotname,string("diago"));
}

int T41PNPActionServer::evalCondition(string cond) {

	int r=-1; // -1 default, conditions will be evaluated by PNPConditionEvent variable

    if (cond == "true"  )  return 1; 

    if (cond.find("L_diago")!=string::npos)
        return 1; // the robot is in the correct position -> always true


    // when this function returns -1, the condition is evaluated from
    // the events published to PNPConditionEvent
    return r;
}

/*
 * ACTIONS
 */

void T41PNPActionServer::turn(string params, bool *run) {
  cout << "### Executing Turn " << params << " ... " << endl;

  float th_deg = atof(params.c_str());
  do_turn(robotname,th_deg,run);
}

void T41PNPActionServer::followcorridor(string params, bool *run) {
  cout << "### Executing Follow Corridor " << params << " ... " << endl;

  double GX,GY;
  if (getLocationPosition(params,GX,GY)) {
      do_follow_corridor(robotname,GX,GY,run);
      // robot oriented towards the goal
      double RX,RY,RTH;
      getRobotPose(robotname,RX,RY,RTH);
      if (params.find("printer")==string::npos) { // if not printer
        double angle = atan2(GY-RY,GX-RX);
        // turn
        do_turn(robotname, DEG(angle), run);
      }
  }
}


void T41PNPActionServer::say(string params, bool *run) {
  cout << "### Executing Say " << params << " ... " << endl;

  shared::Goal hri_goal;
  hri_goal.kind = GOAL_SPEECH;
  hri_goal.param = params;
  hri_pub.publish(hri_goal);

  int sleeptime=6; // *0.5 sec.
  while (*run && sleeptime-->0 && ros::ok())
    ros::Duration(0.5).sleep();

  if (*run)
      cout << "### Finished Say " << params << endl;
  else
      cout << "### Aborted Say " << params << endl;

}


void T41PNPActionServer::advertise(string params, bool *run) {
  cout << "### Executing Advertise " << params << " ... " << endl;

  double GX,GY;
  if (getLocationPosition(params,GX,GY)) {
      do_movebase(robotname,GX,GY,0,run);
  }
  else ROS_WARN("Advertise: Cannot find location %s.",params.c_str());
  
  if (! *run)
    cout << "ABORT"<<endl;

  if (*run) {
    cout << "\033[22;34;1mADVERTISING: " << params << "\033[0m" << endl;

    shared::Goal hri_goal;
    hri_goal.kind = GOAL_SPEECH;
    hri_goal.param = "Advertise!!!";
    hri_pub.publish(hri_goal);
    
    int sleeptime=6; // *0.5 sec.
    while (*run && sleeptime-->0 && ros::ok())
      ros::Duration(0.5).sleep();
  }

  if (*run)
      cout << "### Finished Advertise " << params << endl;
  else
      cout << "### Aborted Advertise " << params << endl;
}

void T41PNPActionServer::advertiseComplex(string params, bool *run) {
  cout << "### Executing AdvertiseComplex " << params << " ... " << endl;

  double GX,GY;
  if (getLocationPosition(params,GX,GY)) {
      do_movebase(robotname,GX,GY,0,run);
  }
  else ROS_WARN("Advertise: Cannot find location %s.",params.c_str());

  if (! *run)
    cout << "ABORT"<<endl;

  if (*run) {
    cout << "\033[22;34;1mADVERTISING: " << params << "\033[0m" << endl;
    
    shared::Goal hri_goal;
    hri_goal.kind = GOAL_ADVERTISE;
    hri_goal.param = "Ad";
    hri_pub.publish(hri_goal);
  }

  int sleeptime=6; // *0.5 sec.
  while (*run && sleeptime-->0 && ros::ok())
      ros::Duration(0.5).sleep();

  if (*run && ros::ok()){
    shared::Goal hri_goal;
    hri_goal.kind = GOAL_SPEECH;
    hri_goal.param = "Do you want to swipe your card?";
    hri_pub.publish(hri_goal);    
  }

  sleeptime=4; // *0.5 sec.
  while (*run && sleeptime-->0 && ros::ok())
      ros::Duration(0.5).sleep();

  if (*run)
      cout << "### Finished AdvertiseComplex " << params << endl;
  else
      cout << "### Aborted AdvertiseComplex " << params << endl;
}

void T41PNPActionServer::interact(string params, bool *run)
{
    cout << "### Executing Interact " << params << " ... " << endl;

    double GX,GY;
    if (getLocationPosition(params,GX,GY)) {
        double RX,RY,RTH,dx;
        getRobotPose(robotname,RX,RY,RTH);
        if (RX<GX) dx=-1; else dx=+1;
        do_movebase(robotname,GX+dx,GY,0,run);  // cannot use the same position as the target
    }
    else ROS_WARN("Advertise: Cannot find location %s.",params.c_str());

    cout << "\033[22;34;1mINTERACTING: " << params << "\033[0m" << endl;

    shared::Goal hri_goal;
    hri_goal.kind = GOAL_INTERACT;
    hri_goal.param = params;
    hri_pub.publish(hri_goal);
    
    int sleeptime=10; // *0.5 sec.
    while (*run && sleeptime-->0)
      ros::Duration(0.5).sleep();
    
    if (*run) {
        cout << "### Finished Interact" << endl;
    } else
        cout << "### Aborted Interact" << endl;
}



void T41PNPActionServer::swipe(string params, bool *run)
{
    cout << "### Executing Swipe action " << params << " ... " << endl;

    shared::Goal hri_goal;
    hri_goal.kind = GOAL_SPEECH;
    hri_goal.param = "Please swipe your card!";
    hri_pub.publish(hri_goal);
    
    int sleeptime=2; // *0.5 sec.
    while (*run && sleeptime-->0 && ros::ok())
      ros::Duration(0.5).sleep();

    if (*run)
        cout << "### Finished Swipe" << endl;
    else
        cout << "### Aborted Swipe" << endl;
}


void T41PNPActionServer::wait(string params, bool *run)
{
    // ROS_INFO_STREAM("### Executing Wait action " << params << " ... ");
/*
    int sleeptime=1; // * 0.2 sec.
    while (*run && sleeptime-->0)
        ros::Duration(0.2).sleep();
*/

    ros::Duration(0.1).sleep();
/*
    if (*run)
        ROS_INFO("### Finished Wait");
    else
        ROS_INFO("### Aborted Wait");
*/
}

void T41PNPActionServer::restart(string params, bool *run)
{
    ROS_INFO_STREAM("### Executing Restart action " << params << " ... ");


    // publish planToExec to start the plan
    string planname = "AUTOGENpolicy";
    std_msgs::String s;
    s.data = planname;
    plantoexec_pub.publish(s); // start the new one

    if (*run)
        cout << "### Finished Wait" << endl;
    else
        cout << "### Aborted Wait" << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mypnpas");

    T41PNPActionServer mypnpas;
    mypnpas.start();
    ros::spin();

    return 0;
}

