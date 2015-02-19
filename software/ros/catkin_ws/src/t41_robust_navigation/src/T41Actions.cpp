#include <tf/transform_listener.h>
#include <shared/topics_name.h>

#include "T41Actions.h"

#include "t11_kb_modeling/GetLocation.h"

#define RAD(a) ((a)/180.0*M_PI)

using namespace std;


std::string robotname="diago";
tf::TransformListener* listener = NULL;

bool getRobotPose(std::string robotname, double &x, double &y, double &th_rad) {
    if (listener==NULL) {
        listener = new tf::TransformListener();
    }

    string src_frame = "/map";
    string dest_frame = "/" + robotname + "/base_frame";
    if (robotname=="") { // local trasnformation
        src_frame = "map";
        dest_frame = "base_link";
    }

    tf::StampedTransform transform;
    try {
        listener->waitForTransform(src_frame, dest_frame, ros::Time(0), ros::Duration(3));
        listener->lookupTransform(src_frame, dest_frame, ros::Time(0), transform);
    }
    catch(tf::TransformException ex) {
        th_rad = 999999;
        ROS_ERROR("Error in tf trasnform %s -> %s\n",src_frame.c_str(), dest_frame.c_str());
        ROS_ERROR("%s", ex.what());
        return false;
    }
    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    th_rad = tf::getYaw(transform.getRotation());

    return true;
}





actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac_movebase = NULL;

#if 0
std::string turn_topic = "turn";

std::string followcorridor_topic = "follow_corridor";
std::string followperson_topic = "follow_person";
std::string askquestion_topic = "interaction";


actionlib::SimpleActionClient<rococo_navigation::TurnAction> *ac_turn = NULL;
actionlib::SimpleActionClient<rococo_navigation::FollowCorridorAction> *ac_followcorridor = NULL;
actionlib::SimpleActionClient<rococo_navigation::FollowPersonAction> *ac_followperson = NULL;
actionlib::SimpleActionClient<coaches_reasoning::AskQuestionAction> *ac_askquestion = NULL;
rococo_navigation::FollowCorridorGoal goal;

string variableValues;
double pedestrianDistance;

void variableCallback(const std_msgs::String::ConstPtr& msg){
    variableValues = msg->data;
}

void start_followcorridor(float GX, float GY, bool *run) {

  if (ac_followcorridor==NULL) {
    // Define the action client (true: we want to spin a thread)
    ac_followcorridor = new actionlib::SimpleActionClient<rococo_navigation::FollowCorridorAction>(followcorridor_topic, true);

    // Wait for the action server to come up
    while(!ac_followcorridor->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for move_base action server to come up");
    }
  }

  // Read time
  double secs =ros::Time::now().toSec();
  while (secs==0) {  // NEEDED OTHERWISE CLOCK WILL BE 0 AND GOAL_ID IS NOT SET CORRECTLY
      ROS_ERROR_STREAM("Time is null: " << ros::Time::now());
      ros::Duration(1.0).sleep();
    secs =ros::Time::now().toSec();
  }

  // Set the goal (MAP frame)
  rococo_navigation::FollowCorridorGoal goal;
  goal.target_X = GX;  goal.target_Y = GY;   // goal
  goal.max_vel = 0.7;  // m/s

  // Send the goal
  ROS_INFO("Sending goal");
  ac_followcorridor->sendGoal(goal);

  // Wait for termination
  double d_threshold=0.5, d=d_threshold+1.0;
  while (!ac_followcorridor->waitForResult(ros::Duration(0.5)) && (*run) && (d>d_threshold)) {
    ROS_INFO("Running...");
    double RX,RY,RTH;
  }

  // Cancel all goals (NEEDED TO ISSUE NEW GOALS LATER)
  ac_followcorridor->cancelAllGoals(); ros::Duration(1).sleep(); // wait 1 sec
}

#endif



void do_movebase(float GX, float GY, float GTh_DEG, bool *run) { // theta in degrees

  if (ac_movebase==NULL) { //create the client only once
    // Define the action client (true: we want to spin a thread)
    ac_movebase = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(TOPIC_MOVE_BASE, true);

    // Wait for the action server to come up
    while(!ac_movebase->waitForServer(ros::Duration(5.0))){
		    ROS_INFO("Waiting for move_base action server to come up");
    }
  }

  // Read time
  double secs =ros::Time::now().toSec();
  while (secs==0) {  // NEEDED OTHERWISE CLOCK WILL BE 0 AND GOAL_ID IS NOT SET CORRECTLY
	  ROS_ERROR_STREAM("Time is null: " << ros::Time::now());
	  ros::Duration(1.0).sleep();
    secs =ros::Time::now().toSec();
  }

  // Set the goal (MAP frame)
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = GX;
  goal.target_pose.pose.position.y = GY;
  goal.target_pose.pose.orientation.z = sin(RAD(GTh_DEG)/2);
  goal.target_pose.pose.orientation.w = cos(RAD(GTh_DEG)/2);

  // Send the goal
  // ROS_INFO("Sending goal");
  ac_movebase->sendGoal(goal);

  // Wait for termination (check distance every delay seconds
  double delay = 0.5;
  double d_threshold=0.5, d=d_threshold+1.0;
  while (!ac_movebase->waitForResult(ros::Duration(delay)) && (*run) && (d>d_threshold)) {
    // ROS_INFO("Running...");
    double RX,RY,RTH;
    if (getRobotPose(robotname, RX, RY, RTH))
      d = fabs(GX-RX)+fabs(GY-RY);
  }

#if 0
  // Print result
  if (!(*run))
    ROS_INFO("External interrupt!!!");
  else if (d<=d_threshold) 
    ROS_INFO("Target reached (Internal check)");
  else if (ac_movebase->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The base failed to reach the move_base goal for some reason");
  else
    ROS_INFO("!move_base goal reached!");
#endif

  // Cancel all goals (NEEDED TO ISSUE NEW GOALS LATER)
  ac_movebase->cancelAllGoals(); ros::Duration(1).sleep(); // wait 1 sec
}




// Action implementation




#if 0


void gotopose(string params, bool *run) {
  cout << "### Executing Gotopose ... " << params << endl;

  float GX = 0.0f;
  float GY = 0.0f;
  float GTh = 0.0f;
  if(params.find("var") != std::string::npos){
      params = "20_1.8_0";
      vector<std::string> params = split(variableValues, ',');
      std::string location = params[1];
      MapPosition* pos = getMapPositionByName(location);
      if(pos==NULL){
          ROS_WARN("Location %s not know, defaulting to 0,0,0", location.c_str());
      }
      else {
          GX = pos->GX;
          GY = pos->GY;
          ROS_INFO("Going to %f %f", GX,GY);
      }
  }
  else {
      int i=params.find("_");
      GX=atof(params.substr(0,i).c_str());
      int j=params.find("_",i+1);
      GY=atof(params.substr(i+1,j).c_str());
      GTh=atof(params.substr(j+1).c_str());
}
  ROS_INFO("GOTOPOSE: %f %f", GX, GY);
  start_followcorridor(GX, GY, run);

  if (*run)
    cout << "### Finished Gotopose " << endl;
  else
    cout << "### Aborted Gotopose  " << endl;
}

#endif

bool getLocationPosition(string loc, double &GX, double &GY) {
    ros::NodeHandle node;
    ros::ServiceClient siteLoc = node.serviceClient<t11_kb_modeling::GetLocation>("/diago/get_location");

    t11_kb_modeling::GetLocation srv;
    srv.request.loc = loc;

    if (siteLoc.call(srv)) {
        GX = srv.response.coords.position.x; GY = srv.response.coords.position.y;
        ROS_INFO_STREAM("Location " << loc << " at " << GX  << " , " << GY);
    }
    else {
        ROS_ERROR_STREAM("Location "<<loc<<" unknown.");
        return false;
    }

    return true;
}

void advertise(string params, bool *run) {
  cout << "### Executing Advertise " << params << " ... " << endl;

  double GX,GY;
  if (getLocationPosition(params,GX,GY)) {
      do_movebase(GX,GY,0,run);
  }
  else ROS_WARN("Advertise: Cannot find location %s.",params.c_str());


  if (*run)
      cout << "### Finished Advertise " << params << endl;
  else
      cout << "### Aborted Advertise " << params << endl;
}

void interact(string params, bool *run)
{
    cout << "### Executing Interact " << params << " ... " << endl;

    double GX,GY;
    if (getLocationPosition(params,GX,GY)) {
        double RX,RY,RTH,dx;
        getRobotPose(robotname,RX,RY,RTH);
        if (RX<GX) dx=-1; else dx=+1;
        do_movebase(GX+dx,GY,0,run);  // cannot use the same position
    }
    else ROS_WARN("Advertise: Cannot find location %s.",params.c_str());

    if (*run)
        cout << "### Finished " << endl;
    else
        cout << "### Aborted " << endl;
}



void swipe(string params, bool *run)
{
    cout << "### Executing Swipe action " << params << " ... " << endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

    if (*run)
        cout << "### Finished Swipe" << endl;
    else
        cout << "### Aborted Swipe" << endl;
}


void wait(string params, bool *run)
{
    cout << "### Executing Wait action " << params << " ... " << endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(10000));

    if (*run)
        cout << "### Finished Wait" << endl;
    else
        cout << "### Aborted Wait" << endl;
}



#if 0
std::map<std::string, std::string> robotSpeach;
void loadRobotSpeach(){
    robotSpeach.insert ( std::pair<std::string, std::string>("pleaseMove","Could you please move?") );
    robotSpeach.insert ( std::pair<std::string, std::string>("readyToFollow","I'm ready to follow you. Say stop when we have arrived") );
    robotSpeach.insert ( std::pair<std::string, std::string>("loadbag","I'm here to help you carry your bag. When ready, load it on the tray.\nDid you load the bag?") );
    robotSpeach.insert ( std::pair<std::string, std::string>("openDoor","Could you please open the door?") );
    robotSpeach.insert ( std::pair<std::string, std::string>("greet","Hello") );
    robotSpeach.insert ( std::pair<std::string, std::string>("thank","Thank you") );
}

void say(string params, bool *run) {
    if (ac_askquestion==NULL) {
      // Define the action client (true: we want to spin a thread)
      ac_askquestion = new actionlib::SimpleActionClient<coaches_reasoning::AskQuestionAction>(askquestion_topic, true);

      // Wait for the action server to come up
      while(!ac_askquestion->waitForServer(ros::Duration(5.0))){
              ROS_INFO("Waiting for move_base action server to come up");
      }
    }
    if(robotSpeach.empty())loadRobotSpeach();
    int i=params.find(".");
    std::string toSay = params.substr(0,i);
    map<std::string, std::string>::iterator result = robotSpeach.find(toSay);
    if(result!=robotSpeach.end()){
        toSay = result->second;
    }
    else {
        toSay = "NOT FOUND!!";
    }
    ROS_INFO("Will say %s", toSay.c_str());
    coaches_reasoning::AskQuestionGoal goal;
    goal.expression = toSay;  // goal
    goal.requiredAnswer = false;
    goal.yesAnswer = "";
    goal.noAnswer = "";

    // Send the goal
    ROS_INFO("Sending goal");
    ac_askquestion->sendGoal(goal);
    ros::Duration(3.0).sleep();
}

void _follow(string params, bool *run, bool stopAtDistance){
    if (ac_followperson==NULL) {
      ac_followperson = new actionlib::SimpleActionClient<rococo_navigation::FollowPersonAction>(followperson_topic, true);

      while(!ac_followperson->waitForServer(ros::Duration(5.0))){
              ROS_INFO("Waiting for follow person action server to come up");
      }
    }
    rococo_navigation::FollowPersonGoal goal;
    goal.target_X = 0;  goal.target_Y = 0;   // unused so far
    goal.max_vel = 0.7;  // m/s

    // Send the goal
    ROS_INFO("Sending goal FOLLOW");
    ac_followperson->cancelAllGoals();
    ros::Duration(1).sleep();
    ac_followperson->sendGoal(goal);

    while (!ac_followperson->waitForResult(ros::Duration(0.5)) && (*run)){
        if(stopAtDistance && pedestrianDistance>0 && pedestrianDistance<2.0){
            break;
        }
        ROS_INFO("Following...");
    }
    ac_followperson->cancelAllGoals();
}

void approach(string params, bool *run){
    _follow(params, run, true);
}

void follow(string params, bool *run) {
    _follow(params, run, false);
}

void wait(string params, bool *run){
    float duration = atof(params.c_str());
    ros::Duration(duration).sleep();
}

void pnpSuccess(string params, bool *run){
    ros::Duration(1).sleep();
}

void turn(string params, bool *run) {
    if (ac_turn==NULL) {
      ac_turn = new actionlib::SimpleActionClient<rococo_navigation::TurnAction>(turn_topic, true);

      while(!ac_turn->waitForServer(ros::Duration(5.0))){
              ROS_INFO("Waiting for turn action server to come up");
      }
    }
    float angle = atof(params.c_str());
    rococo_navigation::TurnGoal goal;

    goal.target_angle = angle;
    goal.absolute_relative_flag = "REL";
    goal.max_ang_vel = 45.0;  // deg/s

    // Send the goal
    ROS_INFO("Sending goal TURN %f", angle);
    ac_turn->cancelAllGoals();ros::Duration(1).sleep();
    ac_turn->sendGoal(goal);

    while (!ac_turn->waitForResult(ros::Duration(0.5)) && (*run)){
        ROS_INFO("Turning...");
    }
    ac_turn->cancelAllGoals();
}
void generalPedestrianCallback(const coaches_msgs::PedestrianInfo::ConstPtr& pedestrianInfo){
    pedestrianDistance = pedestrianInfo->distance;
}

#endif
