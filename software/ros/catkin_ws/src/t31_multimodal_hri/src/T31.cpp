#include "T31.h"

using namespace std;

T31::T31(ros::NodeHandle n) : nodep("~"), node(n) {
  hri_goal_sub = node.subscribe(TOPIC_HRI_GOAL, 10, &T31::hriGoalCallback, this);
  location_sub = node.subscribe(TOPIC_ROBOT_LOCATION, 10, &T31::locationCallback, this);
  tcp_sub = node.subscribe(TOPIC_RCOMMESSAGE, 10, &T31::tcpCallback, this);
  laser_obsmap_sub = node.subscribe(TOPIC_LASER_OBSMAP, 10, &T31::laserobsmapCallback, this);

  feature_pub = node.advertise<shared::Feature>("t31_feature", 100);
  hri_act_pub = node.advertise<t31_multimodal_hri::HRIActuation>("hri_actuation", 100);
  say_stage_pub = node.advertise<std_msgs::String>(TOPIC_STAGE_SAY, 100);
  PNP_cond_pub = node.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 100);

  interactionTimer = node.createTimer(ros::Duration(5), &T31::intTimerCallback, this, true); interactionTimer.stop();
  adTimer = node.createTimer(ros::Duration(3), &T31::adTimerCallback, this, true); adTimer.stop();
  currentInteraction = "";
}

void T31::locationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  robot = msg->pose.pose;
}

void T31::tcpCallback(tcp_interface::RCOMMessage msg) {
    string sm = msg.value;
    if (sm!="") {
        boost::algorithm::to_lower(sm);
        cout << "Received from ASR: " << sm << endl;
        vector<string> toks;
        boost::split(toks,sm,boost::is_any_of("()\" \n\r"));
        if (toks.size()>1) {
            vector<string>::iterator it = toks.begin();
            string pre = *it++; // should be eityher 'button' or 'asr'
            string frame = "-";
            if (pre=="asr") {
                frame = *it++;
            }
            string value = "-";
            for ( ; it!=toks.end(); it++)
                if (*it!="") {
                    value = *it; break; 
                }
            cout << "Tokenize from ASR: " << toks.size() << " - " << pre << " - " << frame << " - " << value << endl;
            std_msgs::String out;
            out.data = value;
            PNP_cond_pub.publish(out);
            cout << "Published PNP condition from ASR: " << out.data << endl;
        }
    }
}

void T31::hriGoalCallback(const shared::Goal::ConstPtr& msg)
{
  std_msgs::String msgOut;
  if (msg->kind == GOAL_ADVERTISE) {
    msgOut.data = "Advertisement: " + msg->param;
    adTimer.setPeriod(ros::Duration(3));
    adTimer.start();
  } else if (msg->kind == GOAL_INTERACT) {
    msgOut.data = "Interacting with " + msg->param;
    interactionTimer.setPeriod(ros::Duration(5));
    interactionTimer.start();
  } else if (msg->kind == GOAL_FOLLOW) {
    msgOut.data = "Follow me to " + msg->loc;
    interactionTimer.setPeriod(ros::Duration(2));
    interactionTimer.start();
  } else if (msg->kind == GOAL_DONE) {
    msgOut.data = "We arrived to " + msg->loc + ", have a nice day!";
    interactionTimer.setPeriod(ros::Duration(3));
    interactionTimer.start();
  } else if (msg->kind == GOAL_SPEECH) {
    msgOut.data = msg->param;
    interactionTimer.setPeriod(ros::Duration(3+msgOut.data.length()/10.));
    interactionTimer.start();
  }
  currentInteraction = msg->param;
  doSay(msgOut);
}

void T31::laserobsmapCallback(laser_analysis::LaserObstacleMap msg) {
/*
stamp: 
  secs: 2078
  nsecs: 200000000
npoints: 8
mx: 1.2499999851
my: -0.184673331911
var: 0.0073012683974
*/

//    cout << "laser obstacle: " << msg << endl;

    bool r = (msg.npoints>5 && msg.mx<1.3);
    string param = "/diago/PNPconditionsBuffer/personhere"; 
    nodep.setParam(param, r?1:0);
}


void T31::doSay(std_msgs::String msg) {
  if (msg.data!="") {
    std::cout << "Say \033[22;35;1m" << msg.data << "\033[0m" << std::endl;
    msg.data += "      ."; // if not clearing balloon, adding a few spaces to enlarge the balloon
  }
  say_stage_pub.publish(msg);  // received by stage to print out say message
  if (false && msg.data!="") {
      std::stringstream ss; ss << "pico2wave -w /tmp/say_out.wav \"" << msg.data << "\"";
      int r1=system(ss.str().c_str());
      int r2=system("aplay /tmp/say_out.wav");
      if (r1<0 || r2<0) {
	  std::cerr << "ERROR in Say procedure (pico2wave+aplay)" << std::endl;
      }
  }
  
  // Possibly add functions to also display what the robot says
  
}
 
 
void T31::doDisplay(std_msgs::String msg) {
  if (msg.data!="") {
    std::cout << "Display \033[22;35;1m" << msg.data << "\033[0m" << std::endl;
    msg.data += "      ."; // if not clearing balloon, adding a few spaces to enlarge the balloon
  }
  say_stage_pub.publish(msg);  // received by stage to print out say message
  if (false && msg.data!="") {
      std::stringstream ss; ss << "pico2wave -w /tmp/say_out.wav \"" << msg.data << "\"";
      int r1=system(ss.str().c_str());
      int r2=system("aplay /tmp/say_out.wav");
      if (r1<0 || r2<0) {
          std::cerr << "ERROR in Say procedure (pico2wave+aplay)" << std::endl;
      }
  }
  
  // Use tcp_interface to send 'display_{text|image|video}_{interactionname}'
}

void T31::intTimerCallback(const ros::TimerEvent&) {
  shared::Feature msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id="diago";
  msg.kind = HRI_DONE;
  msg.location = robot;
  msg.uid = currentInteraction;
  feature_pub.publish(msg);
  
  std_msgs::String msgOut;
  msgOut.data = "";
  doSay(msgOut);   // to delete the say message in stage
  currentInteraction = "";
}

void T31::adTimerCallback(const ros::TimerEvent&) {
  shared::Feature msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id="diago";
  msg.kind = HRI_AD;
  msg.location = robot;
  msg.uid = currentInteraction;
  feature_pub.publish(msg);
  std_msgs::String msgOut;
  msgOut.data = "";
  doSay(msgOut);  // to delete the say message in stage
  currentInteraction = "";
}


void T31::run() {
  ros::spin();
}

