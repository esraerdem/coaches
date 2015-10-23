#include <boost/thread/thread.hpp>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <boost/algorithm/string.hpp>
#include <shared/topics_name.h>
#include <shared/Goal.h>
#include <shared/goalKind.h>
#include <tcp_interface/RCOMMessage.h>

#include "T41PNPAS.h"

T41PNPActionServer::T41PNPActionServer() : PNPActionServer() {

    event_pub = handle.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 10);
    plantoexec_pub = handle.advertise<std_msgs::String>(TOPIC_PLANTOEXEC, 100);
    hri_pub = handle.advertise<shared::Goal>(TOPIC_HRI_GOAL, 10);
    rcom_pub= handle.advertise<tcp_interface::RCOMMessage>(TOPIC_RCOMMESSAGE,10);

    //position_sub = handle.subscribe("odom", 10, &MyPNPActionServer::odom_callback, this);

    register_action("advertise",&T41PNPActionServer::advertise,this);
    register_action("advertiseComplex",&T41PNPActionServer::advertiseComplex,this);
    register_action("swipe",&T41PNPActionServer::swipe,this);
    register_action("interact",&T41PNPActionServer::interact,this);
    register_action("wait",&T41PNPActionServer::wait,this);
    register_action("turn",&T41PNPActionServer::turn,this);
    register_action("goto",&T41PNPActionServer::gotoplace,this);
//    register_action("goto",&T41PNPActionServer::followcorridor,this);
    register_action("say",&T41PNPActionServer::say,this);
    register_action("ask",&T41PNPActionServer::ask,this);
    register_action("display",&T41PNPActionServer::display,this);
    register_action("restart",&T41PNPActionServer::restart,this);
    register_action("start",&T41PNPActionServer::none,this);
    register_action("patrol",&T41PNPActionServer::none,this);

    handle.param("robot_name",robotname,string("diago"));

    bench_togo=1; // variable for got_nextbench action - 1: up corridor, 2: right corridor

    listener = new tf::TransformListener();

    ac_movebase = NULL; ac_turn = NULL; ac_followcorridor = NULL;

}

int T41PNPActionServer::evalCondition(string cond) {

	int r=-1; // -1 default, conditions will be evaluated by PNPConditionEvent variable

    if (cond == "true")  return 1; 

    if (cond.find("L_diago")!=string::npos)
        return 1; // the robot is in the correct position -> always true


    // when this function returns -1, the condition is evaluated from
    // the events published to PNPConditionEvent
    return r;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mypnpas");

    T41PNPActionServer mypnpas;
    mypnpas.start();
    ros::spin();

    return 0;
}

