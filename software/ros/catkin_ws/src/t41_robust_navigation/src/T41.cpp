#include "T41.h"
#include "pnpgenerator.h"

#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

#include <map>
#include <fstream>

#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace boost::algorithm;


#define DEG(a) ((a)*180.0/M_PI)

T41::T41(ros::NodeHandle node) {
  this->node = node;
  low_level_pub = node.advertise<geometry_msgs::PoseStamped>("t41_low_level", 100);
  nav_goal_sub = node.subscribe("t42_nav_goal", 10, &T41::navGoalCallback, this);
  location_sub = node.subscribe("amcl_pose", 10, &T41::amclposeCallback, this);
  pnpstatus_sub = node.subscribe("currentActivePlaces", 10, &T41::PNPplacesCallback, this);

  policy_sub = node.subscribe("t42_policy", 10, &T41::policyCallback, this);
  plantoexec_pub = node.advertise<std_msgs::String>("planToExec", 100);
  policyresult_pub = node.advertise<t41_robust_navigation::PolicyResult>("t41_policy_result", 100);

  service_path_len = node.advertiseService("get_path_len", &T41::getPathLen, this);

  ros::NodeHandle lnode("~");

  lnode.param("robot_name",robotname,std::string("diago"));
  lnode.param("plan_folder",planfolder,std::string("/tmp"));

  cerr << "\033[22;31;1mT41 Plan folder: \033[0m\033[22;32;1m" << planfolder << "\033[0m" << endl;

}

string extractName(string s) {
    string r="";
    int i=s.find("(");
    if (i!=string::npos) {
       r = s.substr(0,i);
       trim(r);
    }
    return r;
}

string extractParams(string s) {
    string r="";
    int i=s.find("(");
    if (i!=std::string::npos) {
        int j = s.find(")",i+1);
        if (j!=std::string::npos) {
            r = s.substr(i+1,j-i-1);
            trim(r);
        }
    }
    return r;
}

void T41::policyCallback(const t41_robust_navigation::Policy::ConstPtr& msg) {

    std::map<string,string> policy;
    std::map<string,string> visited;
    std::map<std::pair<string,string>,string> transition_fn;
    string initial_state = "RobotPos";
    final_state = msg->final_state;
    goalname = msg->goal_name;

    ROS_INFO("Received policy for goal %s. Initial state: %s Final state: %s", goalname.c_str(), initial_state.c_str(), final_state.c_str());
    std::vector<t41_robust_navigation::StatePolicy> p = msg->policy;
    t41_robust_navigation::StatePolicy sa;
    std::vector<t41_robust_navigation::StatePolicy>::iterator it = p.begin();
    while (it!=p.end()) {
        sa = *it++;
        printf("   %s : %s\n", sa.state.c_str(), sa.action.c_str());
        string spred = extractName(sa.state);
        string sparam = extractParams(sa.state);
        string aname = extractName(sa.action);
        string aparam = extractParams(sa.action);
        string action = aname+"_"+aparam;
        policy[sparam] = action;
        transition_fn[make_pair(sparam,action)] = aparam;

        //cout << "### Added policy " << sparam << " -> " << action << endl;
        //cout << "### Added transition " << sparam << "," << action << " -> " << aparam << endl;

    }

    // Generates the PNP
    PNP pnp("policy");
    Place *p0 = pnp.addPlace("init"); p0->setInitialMarking();
    string current_state = initial_state;
    bool cyclic_policy = false;

    while (current_state!=final_state) {
        Place *p1;
        if (visited[current_state]=="") {
            p1 = pnp.addCondition("["+current_state+"]",p0);
            visited[current_state]="true";
        }
        else {
            std::cerr << "ERROR. Cyclic policy!!!" << std::endl;
            cyclic_policy = true;
            break;
        }
        std::cout << "PNPgen::  " << current_state << ": ";
        string action = policy[current_state];
        if (action=="") {
            std::cerr << "ERROR. No action found at this point!!!" << std::endl;
            exit(-1);
        }
        std::cout << action << " -> ";
        Place *p2 = pnp.addAction(action,p1);
        current_state = transition_fn[std::make_pair(current_state,action)];
        if (current_state=="") {
            std::cerr << "ERROR. No successor state found at this point!!!" << std::endl;
            exit(-1);
        }
        std::cout << current_state << std::endl;
        p0 = p2;
    }
    Place *p1 = pnp.addCondition("["+current_state+"]",p0);
    p1->setName("goal");

    if (cyclic_policy) {
        t41_robust_navigation::PolicyResult res;
        res.goal_name = goalname;
        res.feedback = "FAILURE: cyclic policy";
        res.state = initial_state;
        policyresult_pub.publish(res);
        return;
    }


    string planname = "AUTOGENpolicy";
    string pnpfilename = planfolder+"/"+planname+".pnml";
    std::ofstream of(pnpfilename.c_str());
    of << pnp;
    of.close();

    ROS_INFO_STREAM("Saved PNP file " << pnpfilename);

    // publish planToExec to start the plan
    std_msgs::String s;
    s.data = planname;
    plantoexec_pub.publish(s);

}


void T41::PNPplacesCallback(const std_msgs::String::ConstPtr& msg) {
    string current_places = msg->data;
    if (current_places.find("goal")!=string::npos) {
        t41_robust_navigation::PolicyResult res;
        res.goal_name = goalname;
        res.feedback = "SUCCESS";
        res.state = final_state;
        policyresult_pub.publish(res);
    }
}



/*
void T41::locationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  robot_pose = msg->pose.pose;
}*/

void T41::amclposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {

    /*xPos[idx]=msg->pose.pose.position.x;
        yPos[idx]=msg->pose.pose.position.y;
        thetaPos[idx]=tf::getYaw(.orientation);
      */

    robot_pose = msg->pose.pose;

    float x,y,th;
    x = robot_pose.position.x;
    y = robot_pose.position.y;
    th = tf::getYaw(robot_pose.orientation);

    // printf("Robot %d pose : %.1f %.1f %.1f\n",x,y,DEG(th));
}


void T41::navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
// TODO
/* GetPlan seems very slow !
  nav_msgs::GetPlan srv;
  srv.request.start.pose = robot;
  srv.request.start.header.frame_id="map";
  srv.request.goal = *msg;
  srv.request.goal.header.frame_id="map";
  srv.request.tolerance = 2.1;
  ros::ServiceClient planner = node.serviceClient<nav_msgs::GetPlan>("/diago/planner/planner/make_plan");
  if (planner.call(srv)) {
    if (srv.response.plan.poses.empty())
      low_level_pub.publish(msg); // mock-up : sends to move_base_simple/goal
    else {
      geometry_msgs::PoseStamped goal = srv.response.plan.poses.back();
      low_level_pub.publish(goal); // mock-up : sends to move_base_simple/goal
    }
  } else
*/
    low_level_pub.publish(msg); // mock-up : sends to move_base_simple/goal
}

static inline double sq(double v) { return v*v; };

bool T41::getPathLen(t41_robust_navigation::GetPathLen::Request  &req,
		     t41_robust_navigation::GetPathLen::Response &res)
{
  nav_msgs::GetPlan srv;
  srv.request.start = req.from;
  srv.request.start.header.frame_id="map";
  srv.request.goal = req.to;
  srv.request.goal.header.frame_id="map";
  srv.request.tolerance = 2.1;
  ros::ServiceClient planner = node.serviceClient<nav_msgs::GetPlan>("/diago/planner/planner/make_plan");
  if (planner.call(srv)) {
    // Now computes the path length
    double len = 0;
    geometry_msgs::Point prev;
    std::vector<geometry_msgs::PoseStamped>::iterator it = srv.response.plan.poses.begin();
    if (it != srv.response.plan.poses.end()) {
      prev = it->pose.position;
      ++it;
    }
    while (it != srv.response.plan.poses.end()) {
      geometry_msgs::Point cur = it->pose.position;
      len += sqrt(sq(cur.x-prev.x)+sq(cur.y-prev.y));
      ++it;
      prev = cur;
    }
    res.length = len;
    return true;
  } else {
    ROS_WARN("Unable to compute Path");
    return false;
  }
}



void T41::run() {
  ros::spin();
}

