#include "T42.h"

T42::T42(ros::NodeHandle node) {
  this->node = node;
  hri_goal_pub = node.advertise<shared::Goal>("t42_hri_goal", 100);
  nav_goal_pub = node.advertise<geometry_msgs::PoseStamped>("t42_nav_goal", 100);
  goal_set_sub = node.subscribe("t12_goals_set", 10, &T42::goalSetCallback, this);
  location_sub = node.subscribe("t21_robot_location", 10, &T42::locationCallback, this);

  positionTimer = node.createTimer(ros::Duration(1), &T42::positionTimerCallback, this);
  positionUpdated = false;
  positionTimer.stop(); // not enabled till first location is received
}

void T42::goalSetCallback(const shared::AllGoals::ConstPtr& msg)
{
  std::vector<shared::Goal>::const_iterator it = msg->goals.begin();
  //shared::AllGoals::_goals_type::const_iterator it = msg->goals.begin();
  while (it != msg->goals.end()) {
    std::map<std::string, int>::iterator locIt = fixedSites.find(std::string(it->loc));
    int siteID = -1;
    if (locIt == fixedSites.end())
      siteID = newFixedSite(it->loc);
    else
      siteID = locIt->second;
    siteActionReward[siteID][std::string(it->kind)] = it->value;
    siteActionParam[siteID][std::string(it->kind)] = it->param;
    ++it;
  }
  positionTimer.start(); // reactivates the robot if it was sleeping
  plan();
}

int T42::newFixedSite(std::string site) {
  //  std::string site(name);
  t11_kb_modeling::GetLocation srv;
  srv.request.loc = site;
  ros::ServiceClient siteLoc = node.serviceClient<t11_kb_modeling::GetLocation>("/diago/get_location");
  ros::ServiceClient pathLen = node.serviceClient<t41_robust_navigation::GetPathLen>("/diago/get_path_len");
  if (siteLoc.call(srv)) {
    //    std::cout << "adding site " << site << " at " << srv.response.coords.position.x << " x " <<  srv.response.coords.position.y << std::endl;
    fixedSites[site] = sitesLocation.size();
    sitesLocation.push_back(srv.response.coords.position);
    std::vector<float> dist;
    for (int i=0; i<fixedSites.size()-1; ++i) {
      // compute the distance between site i and the new one
      t41_robust_navigation::GetPathLen srv2;
      srv2.request.from.header.frame_id="/diago/map";
      srv2.request.from.pose.position.x = srv.response.coords.position.x;
      srv2.request.from.pose.position.y = srv.response.coords.position.y;
      srv2.request.from.pose.position.z = srv.response.coords.position.z;
      srv2.request.from.pose.orientation.x = 0;
      srv2.request.from.pose.orientation.y = 0;
      srv2.request.from.pose.orientation.z = 0;
      srv2.request.from.pose.orientation.w = 1;
      srv2.request.to.pose.position.x = sitesLocation[i].x;
      srv2.request.to.pose.position.y = sitesLocation[i].y;
      srv2.request.to.pose.position.z = sitesLocation[i].z;
      srv2.request.to.pose.orientation.x = 0;
      srv2.request.to.pose.orientation.y = 0;
      srv2.request.to.pose.orientation.z = 0;
      srv2.request.to.pose.orientation.w = 1;
      if (pathLen.call(srv2)) {
	dist.push_back(srv2.response.length);
	site2siteDistance[i].push_back(srv2.response.length); // suppose the path is reflexive
	//	std::cout << "dist from new to " << i << " is "<<srv2.response.length<<"\n";
      } else {
	//	std::cout << "dist from new to " << i << " is 0\n";
	dist.push_back(0);
	site2siteDistance[i].push_back(0);
      }
    } // for i
    dist.push_back(0); // no distance between i and i
    site2siteDistance.push_back(dist);
    siteActionReward.push_back(std::map<std::string,float>());
    robot2siteDistance.push_back(0); // will be computed when necessary
    if (robot2siteDistance.size() != fixedSites.size()) {
      std::cerr << "Robot-site : " << robot2siteDistance.size() << " != nb-site : " << fixedSites.size() << std::endl;
    }
    siteActionParam.push_back(std::map<std::string,std::string>());
    return fixedSites.size()-1;
  } else {
    ROS_WARN("Unable to obtain location of %s",site.c_str());
    return -1;
  }
}

void T42::locationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  robot = msg->pose.pose.position;
  positionUpdated = true;
  positionTimer.start();
}

double inline sq(double x) { return x*x; }

void T42::positionTimerCallback(const ros::TimerEvent&) {
  if (positionUpdated) {
    positionUpdated = false;
  } else {
    noMoveCallBack();
  }
}

void T42::noMoveCallBack() {
  positionTimer.stop();
  if (plannedAction != "")
    if (sqrt(sq(robot.x-plannedPosition.x)+sq(robot.y-plannedPosition.y)) < 2) {
      // arrived at planned location
      if (plannedAction == GOAL_PATROL) {
	// Nothing to do here, let's replan
      } else if (plannedAction == GOAL_INTERACT) {
	shared::Goal msg;
	msg.loc = plannedSite;
	msg.kind = plannedAction;
	msg.param = plannedParam;
	msg.value = 0;
	hri_goal_pub.publish(msg);
	return;
      } else if (plannedAction == GOAL_ADVERTISE) {
	shared::Goal msg;
	msg.loc = plannedSite;
	msg.kind = plannedAction;
	msg.param = plannedParam;
	msg.value = 0;
	hri_goal_pub.publish(msg);
	return;
      } else if (plannedAction == GOAL_INSPECT) {
	// TODO take a picture
	return;
      } else if (plannedAction == GOAL_ESCORT) {
	shared::Goal msg;
	msg.loc = plannedSite;
	msg.kind = GOAL_FOLLOW;
	msg.param = plannedParam;
	msg.value = 0;
	hri_goal_pub.publish(msg);
	return;
      } else if (plannedAction == GOAL_ESCORT2) {
	shared::Goal msg;
	msg.loc = plannedSite;
	msg.kind = GOAL_DONE;
	msg.param = plannedParam;
	msg.value = 0;
	hri_goal_pub.publish(msg);
	return;
      }
    } // if target reached
  // TODO plan for a new objective ?
  plan();
} // positionTimerCallback(...)

bool T42::updateDistances(int &closestSite) {
  int bestSite = -1;
  float bestDist = 1000000000; // too much
  int i=0;
  for (std::vector<Point>::iterator it = sitesLocation.begin();
       it != sitesLocation.end(); ++it, ++i) {
    float d = sq(robot.x - it->x) + sq(robot.y - it->y);
    if (d < bestDist) {
      bestSite = i;
      bestDist = d;
    }
  }
  if (bestDist <= 1) { // less than 1 meter away for a known site
    bestDist = sqrtf(bestDist);
    closestSite = bestSite;
    /*    std::string bestSiteName="none";
    for (std::map<std::string, int>::iterator it = fixedSites.begin();
	 it != fixedSites.end(); ++it) {
      if (it->second == bestSite) {
	bestSiteName = it->first;
	break;
      }
    }
    std::cout << "Reusing distance based on " << bestSiteName << "(" <<bestDist<< ")\n";*/
    for (std::map<std::string,int>::iterator it = fixedSites.begin();
	 it != fixedSites.end(); ++it) {
      i = it->second;
      robot2siteDistance[i] = bestDist + site2siteDistance[i][bestSite];
      //      std::cout << "Distance to " << it->first << " approx " << robot2siteDistance[i] <<"\n";
    }
  } else {
    closestSite = -1;
    ros::ServiceClient pathLen = node.serviceClient<t41_robust_navigation::GetPathLen>("/diago/get_path_len");
    int i=0;
    for (std::vector<Point>::iterator it = sitesLocation.begin();
	 it != sitesLocation.end(); ++it, ++i) {
      t41_robust_navigation::GetPathLen srv2;
      srv2.request.from.header.frame_id="map";
      srv2.request.from.pose.position.x = robot.x;
      srv2.request.from.pose.position.y = robot.y;
      srv2.request.from.pose.position.z = robot.z;
      srv2.request.from.pose.orientation.x = 0;
      srv2.request.from.pose.orientation.y = 0;
      srv2.request.from.pose.orientation.z = 0;
      srv2.request.from.pose.orientation.w = 1;
      srv2.request.to.pose.position.x = it->x;
      srv2.request.to.pose.position.y = it->y;
      srv2.request.to.pose.position.z = it->z;
      srv2.request.to.pose.orientation.x = 0;
      srv2.request.to.pose.orientation.y = 0;
      srv2.request.to.pose.orientation.z = 0;
      srv2.request.to.pose.orientation.w = 1;
      if (! pathLen.call(srv2)) 
	return false;
      robot2siteDistance[i] = srv2.response.length;
      //      std::cout << "Distance to site " << i << " is " << robot2siteDistance[i] << "\n";
    } // for it in sitesLocation
  } // if unknown position
  //  std::cout << std::endl;
  return true;
} // updateDistances(int&)

void T42::plan() {
  ROS_INFO("Planning...");
  int i=0;
  int closestSite;
  if (!updateDistances(closestSite))
    return;
  //  ros::ServiceClient pathLen = node.serviceClient<t41_robust_navigation::GetPathLen>("/diago/get_path_len");
  Point best = robot;
  int bestSite = -1;
  std::string bestSiteName = "here";
  std::string bestAction = "nothing";
  std::string bestParam = "";
  float expect = 0;
  for (std::vector<Point>::iterator it = sitesLocation.begin();
       it != sitesLocation.end(); ++it, ++i) {
    /*
    t41_robust_navigation::GetPathLen srv2;
    srv2.request.from.header.frame_id="map";
    srv2.request.from.pose.position.x = robot.x;
    srv2.request.from.pose.position.y = robot.y;
    srv2.request.from.pose.position.z = robot.z;
    srv2.request.from.pose.orientation.x = 0;
    srv2.request.from.pose.orientation.y = 0;
    srv2.request.from.pose.orientation.z = 0;
    srv2.request.from.pose.orientation.w = 1;
    srv2.request.to.pose.position.x = it->x;
    srv2.request.to.pose.position.y = it->y;
    srv2.request.to.pose.position.z = it->z;
    srv2.request.to.pose.orientation.x = 0;
    srv2.request.to.pose.orientation.y = 0;
    srv2.request.to.pose.orientation.z = 0;
    srv2.request.to.pose.orientation.w = 1;
    if (! pathLen.call(srv2)) 
      return;
    {
    robot2siteDistance[i] = srv2.response.length;*/
      for (std::map<std::string,float>::iterator itA=siteActionReward[i].begin(); itA!=siteActionReward[i].end(); ++itA) {
	float estimation = itA->second;
	if (robot2siteDistance[i] >= 1)
	  estimation /= sqrt(robot2siteDistance[i]);

	if (estimation > expect) {
	  best = *it;
	  bestAction = itA->first;
	  bestParam = siteActionParam[i][bestAction];
	  bestSite = i;
	  expect = estimation;
	}
      } // for itA in siteActionReward[i]
    //}
  } // for it in sitesLocation
  bestSiteName = look4name(bestSite);
  if (bestAction != GOAL_ESCORT) {
    ROS_INFO("Going to %s for %s %s (%f)", bestSiteName.c_str(), bestAction.c_str(), bestParam.c_str(), expect);
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id="map";
    msg.header.stamp = ros::Time::now();
    msg.pose.position = best;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;
    nav_goal_pub.publish(msg);
    plannedPosition = best;
    plannedAction = bestAction;
    plannedParam = bestParam;
    plannedSite = bestSiteName;
  } else {
    shared::Goal msg;
    ROS_INFO("Inviting %s to follow us to %s (%f)", bestParam.c_str(), bestSiteName.c_str(), expect);
    msg.loc = bestSiteName;
    msg.kind = GOAL_FOLLOW;
    msg.param = plannedParam;
    msg.value = 0;
    hri_goal_pub.publish(msg);
  }
} // plan()

std::string T42::look4name(int siteIdx) {
  std::string name = "unknown";
  for (std::map<std::string, int>::iterator it = fixedSites.begin();
       it != fixedSites.end(); ++it) {
    if (it->second == siteIdx) {
      name = it->first;
      break;
    }
  }
  return name;
}


void T42::run() {
  ros::spin();
}

