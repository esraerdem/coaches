#ifndef __T41PNPAS_H__
#define __T41PNPAS_H__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

#include <pnp_ros/PNPActionServer.h>
#include <rococo_navigation/TurnAction.h>
#include <rococo_navigation/FollowCorridorAction.h>

#include <map>
#include <boost/thread/thread.hpp>



// #include "T41Actions.h"



// Action utilities

/*
bool getRobotPose(std::string robotname, double &x, double &y, double &th_rad);
bool getLocationPosition(std::string loc, double &GX, double &GY);
void do_movebase(std::string robotname, float GX, float GY, float GTh_DEG, bool *run); // theta in degrees
void do_turn(std::string robotname, float GTh_DEG, bool *run);
void do_follow_corridor(std::string robotname, float GX, float GY, bool *run);
*/


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class T41PNPActionServer : public PNPActionServer
{
private:
    ros::NodeHandle handle;
    ros::Publisher event_pub, plantoexec_pub, hri_pub, rcom_pub;
    tf::TransformListener* listener;

    // action clients
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac_movebase;
    actionlib::SimpleActionClient<rococo_navigation::TurnAction> *ac_turn;
    actionlib::SimpleActionClient<rococo_navigation::FollowCorridorAction> *ac_followcorridor;

    std::string robotname;

    boost::mutex mtx_movebase;

    int bench_togo;

public:

    T41PNPActionServer();

    virtual int evalCondition(string cond);

    // Get current robot pose
    bool getRobotPose(string robotname, double &x, double &y, double &th_rad);

    // Get coordinates of semantic location
    bool getLocationPosition(string loc, double &GX, double &GY);

    /*
     * ACTIONS
     */
    void advertise(string params, bool *run);
    void advertiseComplex(string params, bool *run);
    void interact(string params, bool *run);
    void swipe(string params, bool *run);
    void wait(string params, bool *run);
    void turn(string params, bool *run);
    void followcorridor(string params, bool *run);
    void gotoplace(string params, bool *run);
    void say(string params, bool *run);
    void ask(string params, bool *run);
    void display(string params, bool *run);
    void none(string params, bool *run);
    void restart(string params, bool *run);
    
    void do_movebase(float GX, float GY, float GTh_DEG, bool *run);
    void do_turn(float GTh_DEG, bool *run);
    void do_follow_corridor(float GX, float GY, bool *run);

};

#endif
