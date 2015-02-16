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
  plan();
}

#define GOAL_REWARD 20000
//#define ADV_REWARD 15
#define PATIENCE 0.999

struct act {
  int state;
  const std::string *action;
  float reward;
};

void PRUplanner::plan() {
  // Let's focus at first on going to site 3 (phone shop)
  const int TARGET = 3;
  int closestSite;
  if (! updateDistances(closestSite))
    return; // unable to update distances

  std::vector<struct act> actions;
  for (int j=0; j<fixedSites.size(); j++) {
    // go to site j
    for (std::map<std::string,float>::iterator it=siteActionReward[j].begin(); 
	 it != siteActionReward[j].end(); ++it) {
      // and act *it
      struct act cur;
      cur.state = j;
      cur.action = &it->first;
      cur.reward = it->second;
      actions.push_back(cur);
    } // for it
  } // for j

  std::vector<std::vector<float> > durations; // durations[i][j] for acting  j  in state  i
  std::vector<std::vector<float> > rewards; // rewards[i][j] for going action  j  in state i
  std::vector<float> init;
  std::vector<float> initDur;
  std::vector<float> val;
  std::vector<int> act;
  for (int i=0; i<=fixedSites.size(); i++) {
    // from each site i
    std::vector<float> rew;
    std::vector<float> dur;
    bool robotSite = (i==fixedSites.size());
    for (std::vector<struct act>::iterator it=actions.begin(); 
	   it != actions.end(); ++it) {
	// act *it
      int j = it->state;
      if (i==j) { 
	if (i==TARGET) {
	  rew.push_back(GOAL_REWARD); // stay here
	  dur.push_back(60); // 1 minute of interaction
	} else {
	  rew.push_back(0);
	  dur.push_back(60); // 1 minute of do-nothing
	}
      } else if (i==TARGET) {
	rew.push_back(-GOAL_REWARD); // don't leave target
	if (robotSite)
	  dur.push_back(60+robot2siteDistance[j]); // 1 minute of do-nothing + path
	else
	  dur.push_back(60+site2siteDistance[i][j]); // 1 minute of do-nothing + path
      } else {
	if (j==TARGET) {
	  rew.push_back(GOAL_REWARD);
	  if (robotSite)
	    dur.push_back(60+robot2siteDistance[j]); // 1 minute of interaction + path
	  else
	    dur.push_back(60+site2siteDistance[i][j]); // 1 minute of interaction + path
	} else {
	  rew.push_back(it->reward);
	  if (robotSite)
	    dur.push_back(3+robot2siteDistance[j]); // 3 seconds of advertisement + path
	  else
	    dur.push_back(3+site2siteDistance[i][j]); // 3 seconds of advertisement + path
	}
      } // if (i!=j) and (i!=TARGET)
    } // for each it in actions
    val.push_back(0);
    act.push_back(-1);
    rewards.push_back(rew);
    durations.push_back(dur);
  } // for each fixedSites i
  int horizon = val.size()-1; // at most one execution step for each site to visit, excluding current location of the robot
  
  if (val.size() <= 1)
    return; // no action to plan yet
  
  for (int h=0; h<horizon; h++) { // TODO change horizon ?
    bool changed = false;
    std::cout << "h="<<h<<" A=";
    for (int s=0; s<val.size(); s++) {
      float bestR = -1000000;
      float bestS = -1;
      for (int a=0; a<actions.size(); a++) {
	float r = rewards[s][a] + val[actions[a].state];
	r = r * pow(PATIENCE,durations[s][a]);
	if (r > bestR) {
	  bestR = r;
	  bestS = a;
	}
      } // for each action a
      val[s] = bestR;
      act[s] = bestS;
      std::cout << *actions[bestS].action << " in "<<actions[bestS].state << "(" << bestR <<") \t";
    } // for each state s
    std::cout << std::endl;
  } // for each horizon h
  t41_robust_navigation::Policy msg;
  msg.final_state = look4name(TARGET);
  std::vector<t41_robust_navigation::StatePolicy> pol;
  std::cout << "Optimal plan is \n";
  for (int s=0; s<=fixedSites.size(); s++) {
    t41_robust_navigation::StatePolicy stateAction;
    std::string site;
    if (s==fixedSites.size())
      site = "RobotPos";
    else
      site = look4name(s);
    stateAction.state = "location( " + site + " )";
    if (s == TARGET)
      stateAction.action = std::string(GOAL_INTERACT) + "( "+msg.final_state+" )";
    else 
      stateAction.action = *actions[act[s]].action + "( " + look4name(actions[act[s]].state) + " )";
    
    pol.push_back(stateAction);
    std::cout << "Location " << s << " (" << site << ", ";
    if (s==fixedSites.size())
      std::cout << robot.x << " x " << robot.y;
    else
      std::cout << sitesLocation[s].x << " x " << sitesLocation[s].y;
    std::cout << ") : " << *actions[act[s]].action << " in "  << look4name(actions[act[s]].state) << " (" << val[s] << ")" << std::endl;
  } // for s in sites

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
