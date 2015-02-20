#include "shared/topics_name.h"
#include "T41.h"
#include "pnpgenerator.h"

#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>


#include <map>
#include <stack>
#include <fstream>

#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace boost::algorithm;


#define DEG(a) ((a)*180.0/M_PI)

T41::T41(ros::NodeHandle node) {
  this->node = node;
  // low_level_pub = node.advertise<geometry_msgs::PoseStamped>("t41_low_level", 100);
  nav_goal_sub = node.subscribe("t42_nav_goal", 10, &T41::navGoalCallback, this);

  kb_sub = node.subscribe(TOPIC_KB, 10, &T41::kbCallback, this);

  location_sub = node.subscribe(TOPIC_ROBOT_LOCATION, 10, &T41::amclposeCallback, this);
  pnpstatus_sub = node.subscribe(TOPIC_PNPACTIVEPLACES, 10, &T41::PNPplacesCallback, this);

  policy_sub = node.subscribe(TOPIC_POLICY, 10, &T41::policyCallback, this);
  plantoexec_pub = node.advertise<std_msgs::String>(TOPIC_PLANTOEXEC, 100);
  policyresult_pub = node.advertise<t41_robust_navigation::PolicyResult>(TOPIC_POLICY_RESULT, 100);

  pnpcond_pub = node.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 100);

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

string transformParamsWith_(string s) {
    vector<string> v; // conjunction of states
    split(v,s,boost::is_any_of(" ,"),boost::token_compress_on);
    string r="";
    vector<string>::iterator i;
    for (i=v.begin(); i!=v.end(); ) {
        r = r + (*i); i++;
        if (i!=v.end()) r = r+"_";
    }
    return r;
}

string shortPredicate(string s) {
    if (s=="location") return "L";
    else if (s=="desire") return "D";
    else
        return s;
}

string transformState(string s) {
    // s = "p1( X ) & p2 ( X ) & ... "
    //cout << "HERE [" << s << "]" << endl;
    vector<string> v;
    split(v,s,boost::is_any_of("&"),boost::token_compress_on);
    string r="";
    vector<string>::iterator i;
    for (i=v.begin(); i!=v.end(); ) {
        string st = *i; // one state
        //cout << "   - here ["<<st<<"]" << endl;
        string pred = extractName(st);
        string par = transformParamsWith_(extractParams(st));
        r = r + shortPredicate(pred)+"_"+par;
        i++;
        if (i!=v.end()) r = r+"+";
    }
    return r;
}

string extractCondition(string s) {
    // s = "p1( X ) & p2 ( X ) & ... "
    //cout << "HERE [" << s << "]" << endl;
    vector<string> v;
    vector<string> vc;
    split(v,s,boost::is_any_of("&"),boost::token_compress_on);

    vector<string>::iterator i;
    for (i=v.begin(); i!=v.end(); i++) {
        string st = *i; // one state
        //cout << "   - here ["<<st<<"]" << endl;
        string pred = extractName(st);
        if (pred[0]!='_') {
            string par = transformParamsWith_(extractParams(st));
            vc.push_back(shortPredicate(pred)+"_"+par);
        }
    }

    string r="[";
    if (vc.size()==0) {
        r = r + "true";
    }
    if (vc.size()==1) {
        r = r + vc[0];
    }
    else {
        r = r + "and";
        for (i=vc.begin(); i!=vc.end(); i++) {
            r = r + " " + *i;
        }
    }
    r=r+"]";
    return r;
}

int count_conditions(string s) {
    // s = '[cond]' OR '[and c1 c2 ... cn]'
    int r=0;
    std::istringstream is( s );
    std::string line;
    while ( std::getline( is, line, ' ' ) )
        r++;
    if (r>1)
        r--; // remove 'and'
    return r;
}


void T41::policyCallback(const t41_robust_navigation::Policy::ConstPtr& msg) {

    std::map<string,string> policy;
    std::map<string,Place*> visited;
    std::map<std::pair<string,string>,vector<string> > transition_fn;
    std::map<string,string> transformedconditions;

    string initial_state = msg->initial_state;
    final_state = msg->final_state;
    goalname = msg->goal_name;


    ROS_INFO("Received policy for goal %s. Initial state: %s Final state: %s", goalname.c_str(), initial_state.c_str(), final_state.c_str());
    std::vector<t41_robust_navigation::StatePolicy> p = msg->policy;
    t41_robust_navigation::StatePolicy sa;
    std::vector<t41_robust_navigation::StatePolicy>::iterator it = p.begin();
    while (it!=p.end()) {
        sa = *it++;
        // printf("   %s : %s -> ", sa.state.c_str(), sa.action.c_str());

        string tstate = transformState(sa.state);
        transformedconditions[tstate] = extractCondition(sa.state);
        string aname = extractName(sa.action);
        string aparam = transformParamsWith_(extractParams(sa.action));
        string taction = aname+"_"+aparam;

        policy[tstate] = taction;
        //cout << "### Added policy " << tstate << " -> " << taction << endl;

        vector<string> ss = sa.successors;
        vector<string>::iterator is;
        for (is=ss.end(); is!=ss.begin(); ) {
            is--;
            string succ = *is;
            string tsucc = transformState(succ);
            //printf(" %s [%s] ",succ.c_str(),tsucc.c_str());
            transition_fn[make_pair(tstate,taction)].push_back(tsucc);
            //cout << "### Added transition " << tstate << "," << taction << " -> " << tsucc << endl;

        }
        //printf("\n");

    }

    // Generates the PNP
    PNP pnp("policy");
    Place *p0 = pnp.addPlace("init"); p0->setInitialMarking();
    string current_state = transformState(initial_state);
    bool PNPgen_error = false;

    pair<Transition*,Place*> pa = pnp.addCondition(transformedconditions[current_state],p0);
    Place *p1 = pa.second;
    p1->setName(current_state);

    stack< pair<string, Place*> > SK; SK.push(make_pair(current_state,p1));
    while (!SK.empty()) {

        current_state=SK.top().first; Place* current_place = SK.top().second;
        SK.pop();

        std::cout << "PNPgen::  " << current_state << ": ";
        string action = policy[current_state];
        if (action=="") {
            std::cerr << "ERROR. No action found at this point!!!" << std::endl;
            PNPgen_error = true;
            break;
        }
        std::cout << action << " -> ";

        vector<string> vs = transition_fn[std::make_pair(current_state,action)];

        if (vs.size()==0) {
            std::cerr << "ERROR. No successor state found at this point!!!" << std::endl;
            PNPgen_error = true;
            break;
        }

        Place *pe = pnp.addAction(action,current_place);

        vector<string>::iterator iv; int dy=0;

        // Ordering successor states (transitions) wrt number of conditions

        int maxn=0;
//        vector<string> conditions[32];  // max number of atomic conditions
        vector<string> succstates[32];
        for (iv = vs.begin(); iv!=vs.end(); iv++) {
            string succ_state = *iv;
            string cond = transformedconditions[succ_state];
            cout << "Ordering conditions: " << cond << "  ";
            int n = count_conditions(cond);
            if (n>maxn) maxn=n; cout << n << endl;
//            conditions[n].push_back(cond);
            succstates[n].push_back(succ_state);
        }

        for (int k=maxn; k>0; k--) {
//            vector<string> vc = conditions[k];
            vector<string> ss = succstates[k];

            for (int i = 0; i<ss.size(); i++) {
                string succ_state = ss[i];
                std::cout << succ_state << " ";
                Place *ps = visited[succ_state];
                int x = pe->getX(); // x position for all the conditions
                if (ps==NULL) {
                    pair<Transition*,Place*> pa = pnp.addCondition(transformedconditions[succ_state],pe,dy); dy++;
                    Place* pc = pa.second;
                    SK.push(make_pair(succ_state,pc));
                    visited[succ_state]=pc;
                    pc->setName(succ_state);
                    if (succ_state==final_state)
                        pc->setName("goal");
                }
                else {
                    pnp.addConditionBack(transformedconditions[succ_state],pe, ps, dy); dy++;
                }
            } // for i
        } // for k

        std::cout << std::endl;

    }


    if (PNPgen_error) {
        t41_robust_navigation::PolicyResult res;
        res.goal_name = goalname;
        res.feedback = "FAILURE: cannot generate a PNP";
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
    //s.data = "stop";
    //plantoexec_pub.publish(s); // stop the current plan
    //boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    s.data = planname;
    plantoexec_pub.publish(s); // start the new one


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


void T41::kbCallback(const t11_kb_modeling::Knowledge::ConstPtr& msg) {

    if (msg->category==KB_VISIT) {
        // ROS_INFO_STREAM("KB: " << msg->category << " " << msg->knowledge);
        //std_msgs::String s; s.data = msg->knowledge;
        //pnpcond_pub.publish(s);
    }

}


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

