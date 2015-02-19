#include "T42.h"
#include "t41_robust_navigation/policyResults.h"
#include "t41_robust_navigation/PolicyResult.h"
#include "t41_robust_navigation/Policy.h"

#include <boost/format.hpp>

#define DEBUG_VI

#define GOAL_REWARD 20000
//#define ADV_REWARD 15
#define PATIENCE 0.99
#define SPEC_REWARD 200
#define SPEC_DURATION 1
#define SKIP_REWARD 80

#define ROBOT_NAME std::string("diago")

struct state {
  int location;
  std::string name;
};
struct sstate {
  int location;
  std::string name;
  std::string *only_action;
};
struct outcome {
  float p; // probability p 
  int s;   // of reaching state s 
  float r; // with reward r
};
struct act {
  const std::string *action;
  std::vector<struct outcome> outcomes;
  float duration;
};

std::string skipStr = "wait";

class PRUplanner : public T42 {
private:
  // ROS part
  ros::Publisher policy_pub;
  ros::Subscriber result_sub;

  void resultCallback(const t41_robust_navigation::PolicyResult::ConstPtr& msg);

  // Plannig part
  std::vector<struct state> states;
  std::vector<struct sstate> specialStates;
  std::vector<struct act> actions;
  std::vector<struct act> specialActions;
  std::vector<float> values[2];
  std::vector<float> specialValues[2];
  std::vector<std::vector<int> > policy;
  std::vector<std::vector<int> > specialPolicy;

  void chooseAction(int& bestAction, float& bestValue, std::vector<float> values, 
		    std::vector<float> specialValues, int currentState);
  int look4target(std::set<int> &reachableStates, int startState, int stepsLeft,
		  float accReward, int lastAction, float &bestReward, int &bestAction);
  void prepareModel();
  int computePlan(int horizon);
  void publishPlan(int horizon);

public:
  PRUplanner (ros::NodeHandle node);
  virtual void noMoveCallBack();
  virtual void plan();
  void planStochastic();
};

PRUplanner::PRUplanner(ros::NodeHandle node) : T42(node) {
  policy_pub = node.advertise<t41_robust_navigation::Policy>(TOPIC_POLICY, 100);
  result_sub = node.subscribe(TOPIC_POLICY_RESULT, 10, &PRUplanner::resultCallback, this);
}

void PRUplanner::resultCallback(const t41_robust_navigation::PolicyResult::ConstPtr& msg)
{
  if (msg->feedback != POLICY_SUCCESS) {
    // TODO update taboo and replan
  } // if policy has failed
  //plan(); // replan only if executor ask to - otherwise wait for new goals
}

void PRUplanner::noMoveCallBack() {
  //plan();
}

void PRUplanner::plan() {
  int closestSite;
  if (! updateDistances(closestSite))
    return; // unable to update distances
  
  prepareModel();

  if (actions.size() <= 0)
    return; // no action to plan yet

  int horizon = computePlan(10); // 10 actions long ?

  publishPlan(horizon);
}

void PRUplanner::prepareModel() {
  states.clear(); specialStates.clear();
  actions.clear(); specialActions.clear();
  values[0].clear(); specialValues[0].clear();
  values[1].clear(); specialValues[1].clear();
  policy.clear(); specialPolicy.clear();

  for (int j=0; j<fixedSites.size(); j++) {
    // go to site j
    std::string site = look4name(j);
    struct state st = {j, "location( "+ROBOT_NAME+", "+site+" )"};
    states.push_back(st);
    for (std::map<std::string,float>::iterator it=siteActionReward[j].begin(); 
	 it != siteActionReward[j].end(); ++it) {
      // and act *it
      struct act cur;
      cur.action = &it->first;
      cur.duration = siteActionDuration[j][it->first];
      if (*cur.action == GOAL_ADVERTISE_COMPLEX) {
	std::string *act = &siteActionParam[j][it->first];
	struct sstate st2 = {j, "location( "+ROBOT_NAME+", "+site+") & desire( unknown, "+*act+" )",NULL};
	struct outcome res1 = {0.5, j, it->second};
	struct outcome res2 = {0.5, -1-specialStates.size(), it->second};
	specialStates.push_back(st2);
	specialValues[0].push_back(0); // value 0
	specialValues[1].push_back(0); // value 0
	cur.outcomes.push_back(res2);	
	cur.outcomes.push_back(res1);
	struct act spec;
	spec.action = act;
	struct outcome res = {1.0, j, SPEC_REWARD};
	spec.outcomes.push_back(res);
	spec.duration = SPEC_DURATION;
	specialActions.push_back(spec);
      } else if (*cur.action == GOAL_INTERACT) {
	struct sstate st2 = {j, "location( "+ROBOT_NAME+", "+site+") & _goal( )",&skipStr};
	struct outcome res = {1.0, -1-specialStates.size(), it->second};
	specialStates.push_back(st2);
	specialValues[0].push_back(0); // value 0
	specialValues[1].push_back(0); // value 0
	cur.outcomes.push_back(res);
	struct act spec;
	spec.action = &skipStr;
	struct outcome res_ = {1.0, -specialStates.size(), SKIP_REWARD};
	spec.outcomes.push_back(res_);
	spec.duration = 0;
	specialActions.push_back(spec);
      } else {
	struct outcome res = {1.0, j, it->second};
	cur.outcomes.push_back(res);
      }
      actions.push_back(cur);
    } // for it
    values[0].push_back(0); // value 0
    values[1].push_back(0); // value 0
  } // for j

  struct state st = {-1, "location( "+ROBOT_NAME+", RobotPos )"};
  states.push_back(st);
  values[0].push_back(0); // value 0
  values[1].push_back(0); // value 0

#ifdef DEBUG_VI
  std::cout << "States:";
  for (int s=0; s<states.size(); ++s) {
    std::cout << " " << states[s].name << "(" << states[s].location << ")";
  }
  std::cout << " +";
  for (int s=0; s<specialStates.size(); ++s) {
    std::cout << " " << specialStates[s].name << "(" << specialStates[s].location << ")";
  }
  std::cout << std::endl;
  std::cout << "Actions:";
  for (int s=0; s<actions.size(); ++s) {
    std::cout << " " << *actions[s].action;
  }
  std::cout << " +";
  for (int s=0; s<specialActions.size(); ++s) {
    std::cout << " " << *specialActions[s].action;
  }
  std::cout << std::endl;
#endif
}

void PRUplanner::chooseAction(int &bestA, float &bestV, std::vector<float> values, std::vector<float> specialValues, int s) {
  bool robotSite = (s==fixedSites.size());
  int loc,nloc;
  if (robotSite)
    loc = -1;
  else
    if (s<0)
      loc = specialStates[-1-s].location;
    else
      loc = states[s].location;

  int a=0;
  for (std::vector<struct act>::const_iterator it=actions.begin();
       it != actions.end(); ++it,++a) {

    if (s<0)
      if (specialStates[-1-s].only_action != NULL)
	if (specialStates[-1-s].only_action != it->action)
	  continue;

    float expV = 0;
    for (std::vector<struct outcome>::const_iterator itRes=it->outcomes.begin();
	 itRes != it->outcomes.end(); ++itRes) {
      float expO;
      if (itRes->s >= 0) {
	expO = values[itRes->s];   // value of a fixed site
	nloc = states[itRes->s].location;
      } else {
	expO = specialValues[-1-itRes->s]; // value of a special state
	nloc = specialStates[-1-itRes->s].location;
      }
      expO = expO*pow(PATIENCE, it->duration); // expected value minus the task time
      if (robotSite || (s<0)) {
	// current position or special state
	expO += itRes->r;
      } else if (loc != nloc) {
	// new location or special 
	expO += itRes->r;
      } else if (itRes->r>=1000) {
	// staying on final goal
	expO += itRes->r;
      } else {
	expO += -1000000; // much - really forbidden to go to same site twice
      }
      float distance;
      if (robotSite)
	distance = robot2siteDistance[nloc];
      else
	distance = site2siteDistance[loc][nloc];
      expV += itRes->p * expO * pow(PATIENCE, distance);
    } // for *it in it->outcomes
    if (expV > bestV) {
      bestV = expV;
      bestA = a;
    }
  } // for *it in actions
} // chooseActions(...)

int PRUplanner::computePlan(int horizon) {
  for (int h=horizon-1; h>=0; --h) {
    int s;
#ifdef DEBUG_VI
    std::cout << "H"<<h<<":";
#endif
    policy.push_back(std::vector<int>());
    specialPolicy.push_back(std::vector<int>());
    // updates the value of fixed states
    for (s=0; s<=fixedSites.size(); ++s) {
      float bestV = -1000000;
      int bestA = 0;
      chooseAction(bestA,bestV,values[1-h%2],specialValues[1-h%2],s);
      values[h%2][s] = bestV;
      policy.back().push_back(bestA);
#ifdef DEBUG_VI
      std::cout << " "<<bestA<<"("<<bestV<<")";
#endif
    } // for state s
#ifdef DEBUG_VI
    std::cout << " +";
#endif
    // updates the values of special states
    for (s=0; s<specialStates.size(); ++s) {
      float bestV = -1000000;
      int bestA = 0;
      actions.push_back(specialActions[s]);
      chooseAction(bestA,bestV,values[1-h%2],specialValues[1-h%2],-1-s);
      actions.pop_back();
      if (bestA>=actions.size())
	bestA = -1 - s;
      specialValues[h%2][s] = bestV;
      specialPolicy.back().push_back(bestA);
#ifdef DEBUG_VI
      std::cout << " "<<bestA<<"("<<bestV<<")";
#endif
    } // for s
#ifdef DEBUG_VI
    std::cout << std::endl;
#endif
  } // for h
  return horizon;
} // computePlan(h)

void PRUplanner::publishPlan(int horizon) {
  t41_robust_navigation::Policy msg;
  float bestReward = 0;
  int TARGETaction = 0;
  std::set<int> reachableStates;
  int TARGETstate = look4target(reachableStates, fixedSites.size(), horizon-1, 0, 0, bestReward, TARGETaction);

  if (TARGETstate<0) {
    msg.final_state = specialStates[-1-TARGETstate].name;
  } else {
    msg.final_state = states[TARGETstate].name;
  }

  std::vector<t41_robust_navigation::StatePolicy> pol;
  std::cout << "Optimal plan is \n";
  std::string ostep = " & _step( 0 )";
  for (int steps=0; steps<horizon; ++steps) {
    std::string cstep = str(boost::format(" & _step( %d )") %steps);
    for (int s=0; s<states.size(); ++s) {
      t41_robust_navigation::StatePolicy stateAction;
      stateAction.state = states[s].name + cstep;
      struct act *a;
      if (policy[steps][s] >= 0)
	a = &actions[policy[steps][s]];
      else
	a = &specialActions[-1-policy[steps][s]];
      
      int dest = a->outcomes[0].s;
      if (dest < 0)
	stateAction.action = *a->action + "( " 
	  + look4name(specialStates[-1-dest].location) + " )";
      else
	stateAction.action = *a->action + "( " 
	  + look4name(states[dest].location) + " )";
      
      for (std::vector<struct outcome>::const_iterator itRes=a->outcomes.begin();
	   itRes != a->outcomes.end(); ++itRes) {
	if (itRes->s < 0)
	  stateAction.successors.push_back(specialStates[-1-itRes->s].name + ostep);
	else
	  stateAction.successors.push_back(states[itRes->s].name + ostep);
      } // for *itRes in successors of a
      pol.push_back(stateAction);
      std::cout << s << " - " << stateAction.state ;
      std::cout << ": " << stateAction.action << " (" << values[0][s] << ")" << std::endl;
    } // for s in sites
    for (int s=0; s<specialStates.size(); s++) {
      t41_robust_navigation::StatePolicy stateAction;
      stateAction.state = specialStates[s].name + cstep;
      struct act *a;
      if (specialPolicy[steps][s] >= 0)
	a = &actions[specialPolicy[steps][s]];
      else
	a = &specialActions[-1-specialPolicy[steps][s]];

      int dest = a->outcomes[0].s;
      if (dest < 0)
	stateAction.action = *a->action + "( " 
	  + look4name(specialStates[-1-dest].location) + " )";
      else
	stateAction.action = *a->action + "( " 
	  + look4name(states[dest].location) + " )";

      for (std::vector<struct outcome>::const_iterator itRes=a->outcomes.begin();
	   itRes != a->outcomes.end(); ++itRes) {
	if (itRes->s < 0)
	  stateAction.successors.push_back(specialStates[-1-itRes->s].name + ostep);
	else
	  stateAction.successors.push_back(states[itRes->s].name + ostep);
      } // for *itRes in successors of a
      pol.push_back(stateAction);
      std::cout << (-1-s) << " - " << stateAction.state ;
      std::cout << ": " << stateAction.action << " (" << specialValues[0][s] << ")" << std::endl;
    } // for s in sites
    ostep = cstep;
  } // for horizon h

  msg.policy = pol;

  msg.initial_state = str(boost::format("%s & _step( %d )") %states.back().name %(horizon-1)); // RobotPos is the last "fixed" state
  if (TARGETaction<0)
    msg.goal_name = *specialActions[-1-TARGETaction].action;
  else
    msg.goal_name = *actions[TARGETaction].action;
  if (TARGETstate<0)
    msg.goal_name +=  "( " + look4name(specialStates[-1-TARGETstate].location) + " )";
  else
    msg.goal_name +=  "( " + look4name(states[TARGETstate].location) + " )";
  policy_pub.publish(msg);

} // publishPlan(h)

int PRUplanner::look4target(std::set<int> &reachableStates, int startState, int stepsLeft,
			    float accReward, int lastAction, float &bestReward, int &bestAction) {
  int action;
  int bestState = startState;
  if (startState==0) {
    if (accReward>bestReward) {
      bestState = startState;
      bestReward = accReward;
      bestAction = lastAction;
    }
    return bestState;
  } else if (startState<0) {
    action = specialPolicy[stepsLeft][-1-startState];
  } else {
    action = policy[stepsLeft][startState];
  }
  struct act *act;
  if (action<0)
    act = &specialActions[-1-action];
  else
    act = &actions[action];
  for (std::vector<struct outcome>::const_iterator it=act->outcomes.begin();
       it != act->outcomes.end(); ++it) {
    if (reachableStates.insert(it->s).second) {
      // new reachable state
      bestState = look4target(reachableStates, it->s, stepsLeft-1, accReward+it->r*it->p, action, bestReward, bestAction);
    } else {
      // we got back to some state, let's check if this path is better than previous one...
      if (accReward>bestReward) {
	bestState = startState;
	bestReward = accReward;
	bestAction = lastAction;
      }
    }
  } // for *it in action.outcomes
  return bestState;
} // look4target


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pruPlanner");

  ros::NodeHandle node;

  PRUplanner t42(node);
  t42.run();

  return 0;
}
