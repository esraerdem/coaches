/**
 * Mock-up module that sends shop-sites with patrol values every 15s
 * Implements the get_location service
 * Receives people events and builds goals from them
 * Sends goals
 */

#include "ros/ros.h"
#include "shared/Feature.h"
#include "shared/featureKind.h"
#include "shared/topics_name.h"
#include "t11_kb_modeling/GetLocation.h"
#include "t11_kb_modeling/GetAllSites.h"
#include "t11_kb_modeling/Knowledge.h"
#include "t11_kb_modeling/knowledgeKind.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <sstream>
#include <istream>
#include <map>

#define PATROL_RATE (1/15.)

// TODO: read for ROS param...
#define ROBOTNAME "diago"


using namespace std;

typedef std::map<std::string, geometry_msgs::Point> lmap;
typedef std::map<std::string, std::string> smap;
typedef std::map<std::pair<std::string, std::string>,bool> bmap2;

class KB {
public:
    // *** functional fluents ***

    // location ( agent ) = name
    smap location;

    // at ( agent ) = X, Y
    lmap at;

    // *** predicates ***

    // desire ( agent, something )
    bmap2 desire;
};



std::ostream& operator<< (std::ostream& stream, const KB& kb) {
    smap::const_iterator il = kb.location.begin();
    while (il!=kb.location.end()) {
        stream << "location(" << il->first << ") = " << il->second << endl;
        il++;
    }

    return stream;
}


class MockModel {
  private:
  ros::NodeHandle node;
  ros::Publisher knowledge_pub;
  ros::Subscriber position_sub, hri_feature_sub, env_feature_sub;
  ros::ServiceServer service_get_location, service_get_all_sites;
  lmap locations;
  lmap peoples;
  KB kb;


  void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void hriFeatureCallback(const shared::Feature::ConstPtr& msg);
  void envFeatureCallback(const shared::Feature::ConstPtr& msg);

  bool getLocation(t11_kb_modeling::GetLocation::Request  &req,
		   t11_kb_modeling::GetLocation::Response &res);

  bool getAllSites(t11_kb_modeling::GetAllSites::Request  &req,
		   t11_kb_modeling::GetAllSites::Response &res);

  public:
  MockModel(ros::NodeHandle node);
  void run();
};

static inline geometry_msgs::Point mkPoint(float x,float y) {
  geometry_msgs::Point res;
  res.x = x;
  res.y = y;
  res.z = 0;
  return res;
}

MockModel::MockModel(ros::NodeHandle node) {
  this->node = node;
  knowledge_pub = node.advertise<t11_kb_modeling::Knowledge>(TOPIC_KB, 100);

  position_sub = node.subscribe(TOPIC_ROBOT_LOCATION, 100, &MockModel::positionCallback, this);
  hri_feature_sub = node.subscribe("t31_feature", 10, &MockModel::hriFeatureCallback, this);
  env_feature_sub = node.subscribe("t21_feature", 10, &MockModel::envFeatureCallback, this);
  
  service_get_location = node.advertiseService(SERVICE_GET_LOCATION, &MockModel::getLocation, this);
  service_get_all_sites = node.advertiseService(SERVICE_GET_ALL_SITES, &MockModel::getAllSites, this);

  ros::NodeHandle lnode("~");
  std::string mapName;
  lnode.param("map_name",mapName,std::string("none"));
  std::cout << "Requested building mockup for " << mapName << std::endl;

  if (mapName != "DISB1") {
  std::cout << "Inserting RiveDeL'Orne sites...\n";
  locations["doorWest"]=mkPoint(6,19);
  locations["doorEast"]=mkPoint(93.69,29.75);
  locations["monoprix"]=mkPoint(78,24);
  locations["phone"]=mkPoint(89,31);
  locations["restaurant"]=mkPoint(10,23);
  locations["carPark"]=mkPoint(27,20);
  }
  
  if (mapName != "RiveOrne") {
  std::cout << "Inserting DIAG site...\n";
  //Printer DIAG Demo
  locations["home"]=mkPoint(1.3,15);
  locations["printer"]=mkPoint(3,21.5);
  locations["technician"]=mkPoint(1.6,1.3);
  locations["office"]=mkPoint(1.2,19.5);
  locations["office1"]=mkPoint(1.2,23.9);
  locations["office2"]=mkPoint(1.3,19.5);
  locations["office3"]=mkPoint(1.3,15.1);
  locations["office4"]=mkPoint(1.1,10.65);  
  locations["office5"]=mkPoint(0.8,6.4);
  locations["office6"]=mkPoint(0.8,1.8);
  locations["corner"]=mkPoint(1.8,1.8);
  locations["bench1"]=mkPoint(3.0,8.0);
  locations["bench2"]=mkPoint(8.0,3.0);
  }
  std::cout << "Done.\n" << std::endl;
} // constructor

static inline double sq(double x) {return x*x;}

void MockModel::positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  geometry_msgs::Point robot = msg->pose.pose.position;
  ROS_DEBUG("Robot at %f %f",robot.x, robot.y);
  kb.at[ROBOTNAME] = mkPoint(robot.x, robot.y);
  lmap::iterator it = locations.begin(); 
  bool sent = false;
  while (it != locations.end()) {
    if (sq(robot.x - it->second.x)+sq(robot.y - it->second.y) < 1) {
      // ROS_INFO("KB: Visited site %s",it->first.c_str());
      t11_kb_modeling::Knowledge kb_msg;
      kb_msg.header.frame_id=ROBOTNAME;
      kb_msg.header.stamp = ros::Time::now();
      kb_msg.category = KB_VISIT;
      kb_msg.knowledge = it->first;
      knowledge_pub.publish(kb_msg);
      sent = true;
      kb.location[ROBOTNAME] = it->first; // put this predicate in the KB
    }
    ++it;
  } // while it in locations
  if (! sent) {
    t11_kb_modeling::Knowledge kb_msg;
    kb_msg.header.frame_id=ROBOTNAME;
    kb_msg.header.stamp = ros::Time::now();
    kb_msg.category = KB_ROBOT_AT;
    std::ostringstream strs;
    strs << robot.x << "\t" << robot.y;
    kb_msg.knowledge = strs.str();
    knowledge_pub.publish(kb_msg);
    kb.location[ROBOTNAME] = ""; // remove predicate from the KB (robot is not anymore at previous location)
  }
}

bool MockModel::getAllSites(t11_kb_modeling::GetAllSites::Request  &req,
		      t11_kb_modeling::GetAllSites::Response &res)
{
  std::vector<std::string> names;
  lmap::iterator it = locations.begin(); 
  while (it != locations.end()) {
    names.push_back(it->first);
    ++it;
  } // while it in locations
  res.ids = names;
  return true;
}

bool MockModel::getLocation(t11_kb_modeling::GetLocation::Request  &req,
		      t11_kb_modeling::GetLocation::Response &res)
{
  ROS_DEBUG("Requesting %s", req.loc.c_str());
  std::string id(req.loc);
  lmap::iterator it,end;
  
  if (id == "office") {
      id = "office5";
  }

  
  if (id.substr(0, 3) == "_P_") {
    it = peoples.find(id);
    res.fixed = false;
    end = peoples.end();
  } else {
    it = locations.find(id);
    res.fixed = true;
    end = locations.end();
  }
  if (it != end) {
    res.coords.position = it->second;
    res.coords.orientation.x = 0;
    res.coords.orientation.y = 0;
    res.coords.orientation.z = 0;
    res.coords.orientation.w = 1;
    return true;
  } else {
    ROS_WARN("location %s is unknown",id.c_str());
    return false;
  }
}

void MockModel::hriFeatureCallback(const shared::Feature::ConstPtr& msg)
{
// TODO
}

void MockModel::envFeatureCallback(const shared::Feature::ConstPtr& msg)
{
  if (msg->kind == DETECT_PEOPLE) {
    peoples["_P_"+msg->uid] = msg->location.position; // TODO deal with deletion of people ?
    t11_kb_modeling::Knowledge kb;
    kb.header.frame_id="diago";
    kb.header.stamp = ros::Time::now();
    kb.category = KB_PEOPLE;
    kb.knowledge = msg->uid;

    knowledge_pub.publish(kb);
  }
}


// r t21_robot_location:=/diago/amcl_pose __ns:=/diago

void MockModel::run() {
  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mockModel");

  ros::NodeHandle node;

  MockModel mock(node);
  mock.run();

  return 0;
}

