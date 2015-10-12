#ifndef __T41PNPAS_H__
#define __T41PNPAS_H__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <pnp_ros/PNPActionServer.h>

#include "T41Actions.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class T41PNPActionServer : public PNPActionServer
{
private:
    ros::NodeHandle handle;
    ros::Publisher event_pub, plantoexec_pub, hri_pub;
    // ros::Subscriber laser_sub, variable_sub, speech_sub, pedestrian_sub, general_Pedestrian_sub;
    std::string robotname;

public:

    T41PNPActionServer();

    virtual int evalCondition(string cond);

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
    void say(string params, bool *run);
    void none(string params, bool *run);
    void restart(string params, bool *run);

};

#endif
