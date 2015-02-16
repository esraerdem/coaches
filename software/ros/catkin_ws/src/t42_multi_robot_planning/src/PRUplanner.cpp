#include "T42.h"
#include "t41_robust_navigation/policyResults.h"
#include "t41_robust_navigation/PolicyResult.h"
#include "t41_robust_navigation/Policy.h"

class PRUplanner : public T42 {
private:
  ros::Publisher policy_pub;
  ros::Subscriber result_sub;

  void resultCallback(const t41_robust_navigation::PolicyResult::ConstPtr& msg);

public:
  PRUplanner (ros::NodeHandle node);
  virtual void noMoveCallBack();
  virtual void plan();
};

PRUplanner::PRUplanner(ros::NodeHandle node) : T42(node) {
  policy_pub = node.advertise<t41_robust_navigation::Policy>("t42_policy", 100);
  result_sub = node.subscribe("t41_policy_result", 10, &PRUplanner::resultCallback, this);
}

void PRUplanner::resultCallback(const t41_robust_navigation::PolicyResult::ConstPtr& msg)
{
  if (msg->feedback != POLICY_SUCCESS) {
    // TODO update taboo and replan
  } // if policy has failed
}

void PRUplanner::noMoveCallBack() {
  //plan();
}

#define GOAL_REWARD 200
#define ADV_REWARD 50
#define PATIENCE 0.999

void PRUplanner::plan() {
  // Let's focus at first on going to site 3 (phone shop)
  const int TARGET = 3;
  int closestSite;
  if (! updateDistances(closestSite))
    return; // unable to update distances

  std::vector<std::vector<float> > durations; // durations[i][j] for going from i to j
  std::vector<std::vector<float> > rewards; // rewards[i][j] for going from i to j
  std::vector<float> init;
  std::vector<float> initDur;
  std::vector<float> val;
  std::vector<int> act;
  for (int i=0; i<fixedSites.size(); i++) {
    std::vector<float> rew;
    std::vector<float> dur;
    for (int j=0; j<fixedSites.size(); j++) {
      if (i==j) 
	if (i==TARGET) {
	  rew.push_back(GOAL_REWARD); // stay here
	  dur.push_back(60); // 1 minute of interaction
	} else {
	  rew.push_back(0);
	  dur.push_back(60); // 1 minute of do-nothing
	}
      else if (i==TARGET) {
	rew.push_back(-GOAL_REWARD); // don't leave target
	dur.push_back(60+site2siteDistance[i][j]); // 1 minute of do-nothing + path
      } else {
	if (j==TARGET) {
	  rew.push_back(GOAL_REWARD);
	  dur.push_back(60+site2siteDistance[i][j]); // 1 minute of interaction + path
	} else {
	  rew.push_back(ADV_REWARD);
	  dur.push_back(3+site2siteDistance[i][j]); // 3 seconds of advertisement + path
	}
      } // if (i!=j)
    } // for each fixedSites j
    if (closestSite<0) {
      // If robot not on a site
      if (i==TARGET) {
	rew.push_back(-GOAL_REWARD); // leaving the goal to go at robot's
	dur.push_back(60+robot2siteDistance[i]); // 1 minute of do-nothing + path
	init.push_back(GOAL_REWARD);
	initDur.push_back(60+robot2siteDistance[i]); // 1 minute of interaction + path
      } else {
	rew.push_back(0); // not going back to start!
	dur.push_back(60+robot2siteDistance[i]); // 1 minute of do-nothing + path
	init.push_back(ADV_REWARD);
	initDur.push_back(3+robot2siteDistance[i]); // 3 seconds of adertisement + path
      }
    }
    val.push_back(0);
    act.push_back(-1);
    rewards.push_back(rew);
    durations.push_back(dur);
  } // for each fixedSites i
  int horizon = val.size(); // at most one execution step for each site to visit, excluding current location of the robot
  if (closestSite<0) {
    // If robot not on a site
    init.push_back(0);
    initDur.push_back(60); // 1 minute of do-nothing
    val.push_back(0);
    act.push_back(-1);
    rewards.push_back(init); // last state is current position
    durations.push_back(initDur);
  }
  
  if (val.size() <= 1)
    return; // no action to plan yet
  
  for (int h=0; h<horizon; h++) { // TODO change horizon ?
    bool changed = false;
    std::cout << "h="<<h<<" A=";
    for (int s=0; s<val.size(); s++) {
      float bestR = -1000000;
      float bestS = -1;
      for (int s2=0; s2<val.size(); s2++) {
	float r = rewards[s][s2] + val[s2];
	r = r * pow(PATIENCE,durations[s][s2]);
	if (r > bestR) {
	  bestR = r;
	  bestS = s2;
	}
      } // for each state s2
      val[s] = bestR;
      act[s] = bestS;
      std::cout << bestS << "(" << bestR <<") \t";
    } // for each state s
    std::cout << std::endl;
  } // for each horizon h
  t41_robust_navigation::Policy msg;
  msg.final_state = look4name(TARGET);
  std::vector<t41_robust_navigation::StatePolicy> pol;
  std::cout << "Optimal plan is \n";
  for (int s=0; s<fixedSites.size(); s++) {
    std::string site = look4name(s);
    t41_robust_navigation::StatePolicy stateAction;
    stateAction.state = "location( " + site + " )";
    if (s == TARGET)
      stateAction.action = std::string(GOAL_INTERACT) + "( "+msg.final_state+" )";
    else {
      if (act[s] == TARGET)
	stateAction.action = GOAL_INTERACT;
      else
	stateAction.action = GOAL_ADVERTISE;
      stateAction.action += "( " + look4name(act[s]) + " )";
    }
    pol.push_back(stateAction);
    std::cout << "Location " << s << " (" << look4name(s) << ", " 
	      << sitesLocation[s].x << " x " << sitesLocation[s].y
	      << ") : Goto " << act[s] << " (" << val[s] << ")" << std::endl;
  }
  t41_robust_navigation::StatePolicy stateAction;
  stateAction.state = "location( RobotPos )";
  if (closestSite<0) {
    // If robot not on a site
    int s = act.back();
    if (s == TARGET)
      stateAction.action = GOAL_INTERACT;
    else
      stateAction.action = GOAL_ADVERTISE;
    stateAction.action += "( " + look4name(s) + " )";
    
    std::cout << "Robot at " << robot.x << " x " << robot.y
	      << " : Goto " << s << " (" << val.back() << ")" << std::endl;
  } else {
    if (closestSite == TARGET)
      stateAction.action = std::string(GOAL_INTERACT) + "( " + msg.final_state + " )";
    else { // closest site is not the target
      int s = act[closestSite];
      if (s == TARGET)
	stateAction.action = GOAL_INTERACT;
      else
	stateAction.action = GOAL_ADVERTISE;
      stateAction.action += "( " + look4name(s) + " )";
    }
    std::cout << "Robot is at location " << closestSite << " ("
	      << look4name(closestSite) << ")" << std::endl;
  }
  pol.push_back(stateAction);
  msg.policy = pol;
  policy_pub.publish(msg);
} // plan()


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pruPlanner");

  ros::NodeHandle node;

  PRUplanner t42(node);
  t42.run();

  return 0;
}
