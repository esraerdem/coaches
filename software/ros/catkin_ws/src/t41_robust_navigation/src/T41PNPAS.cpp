#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
//#include <coaches_msgs/Speech.h>
//#include <coaches_msgs/PedestrianInfo.h>
#include <boost/algorithm/string.hpp>

#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_ros/PNPActionServer.h>

#include "T41Actions.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

extern std::string robotname; // defined in T41Actions.cpp



class T41PNPActionServer : public PNPActionServer
{
private:
    ros::NodeHandle handle;
    ros::Publisher event_pub;
    ros::Subscriber laser_sub, variable_sub, speech_sub, pedestrian_sub, general_Pedestrian_sub;
public:

    T41PNPActionServer() : PNPActionServer()
    {
        event_pub = handle.advertise<std_msgs::String>("PNPConditionEvent", 10);
        //position_sub = handle.subscribe("odom", 10, &MyPNPActionServer::odom_callback, this);

        register_action("advertise",&advertise);
        register_action("advertiseComplex",&advertise);
        register_action("swipe",&fake);
        register_action("interact",&interact);

        handle.param("robot_name",robotname,string("diago"));
    }
    

    virtual int evalCondition(string cond) {

        // printf("-- Evaluating condition %s \n",cond.c_str());

        if (cond=="RobotPos")
            return 1; // the robot is in its own position -> always true

        // return 1;   // Test, all conditions are true!!!

        return PNPActionServer::evalCondition(cond);  // default returns -1

        // when this function returns -1, the condition is evaluated from
        // the events published to PNPConditionEvent

    }





/*
    void odom_callback(const sensor_msgs::Odom::ConstPtr& msg)
    {
        std::vector<float> scans;
        scans=std::vector<float>(msg->ranges);
        if (scans[scans.size()/2]<1.0) {
            std_msgs::String cond;
            cond.data = "obstacle";
            event_pub.publish(cond);
        }
    }
*/

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mypnpas");

    T41PNPActionServer mypnpas;
    mypnpas.start();
    ros::spin();

    return 0;
}

