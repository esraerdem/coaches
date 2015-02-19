#include "T42.h"
#include "t41_robust_navigation/policyResults.h"
#include "t41_robust_navigation/PolicyResult.h"
#include "t41_robust_navigation/Policy.h"

#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#define DEBUG_VI

#define GOAL_REWARD 20000
//#define ADV_REWARD 15
#define PATIENCE 0.99
#define SPEC_REWARD 200
#define SPEC_DURATION 1
#define SKIP_REWARD 80

struct simpleAct {
  int state;
  const std::string *action;
  float reward;
};
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
  ros::Publisher policy_pub;
  ros::Subscriber result_sub;

  void resultCallback(const t41_robust_navigation::PolicyResult::ConstPtr& msg);
  void chooseAction(int& bestAction, float& bestValue, std::vector<struct act>& actions, 
		    std::vector<float>& values, std::vector<float>& specialValues,
		    std::vector<struct state>& states, std::vector<struct sstate>& specialStates, int currentState);
  int look4target(std::vector<struct act>& actions, std::vector<struct act>& specialActions, 
		  std::vector<float>& values, std::vector<float>& specialValues,
		  std::vector<std::vector<int> >& policy, std::vector<std::vector<int> >& specialPolicy,
		  std::vector<struct state>& states, std::vector<struct sstate>& specialStates,
		  std::set<int> &reachableStates, int startState, int stepsLeft,
		  float accReward, int lastAction, float &bestReward, int &bestAction);
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
  planStochastic(); return;

  int closestSite;
  if (! updateDistances(closestSite))
    return; // unable to update distances

  int TARGET = 0;
  float targetReward = 0;

  std::vector<struct simpleAct> actions;
  for (int j=0; j<fixedSites.size(); j++) {
    // go to site j
    for (std::map<std::string,float>::iterator it=siteActionReward[j].begin(); 
	 it != siteActionReward[j].end(); ++it) {
      // and act *it
      struct simpleAct cur;
      cur.state = j;
      cur.action = &it->first;
      cur.reward = it->second;
      if (cur.reward>targetReward) {
	targetReward = cur.reward;
	TARGET = actions.size();
      }
      actions.push_back(cur);
    } // for it
  } // for j

  int TARGETstate = actions[TARGET].state;

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
    for (std::vector<struct simpleAct>::iterator it=actions.begin(); 
	   it != actions.end(); ++it) {
	// act *it
      int j = it->state;
      if (i==j) { 
	if (i==TARGETstate) {
	  rew.push_back(it->reward); // stay here
	  dur.push_back(60); // 1 minute of interaction
	} else {
	  rew.push_back(0);
	  dur.push_back(60); // 1 minute of do-nothing
	}
      } else if (i==TARGETstate) {
	rew.push_back(-GOAL_REWARD); // don't leave target
	if (robotSite)
	  dur.push_back(60+robot2siteDistance[j]); // 1 minute of do-nothing + path
	else
	  dur.push_back(60+site2siteDistance[i][j]); // 1 minute of do-nothing + path
      } else {
	if (j==TARGETstate) {
	  rew.push_back(it->reward);
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
  msg.final_state = look4name(TARGETstate);
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
    stateAction.action = *actions[act[s]].action + "( " + look4name(actions[act[s]].state) + " )";

    stateAction.successors.push_back("location( " + look4name(actions[act[s]].state) + " )");
    pol.push_back(stateAction);
    std::cout << "Location " << s << " (" << site << ", ";
    if (s==fixedSites.size())
      std::cout << robot.x << " x " << robot.y;
    else
      std::cout << sitesLocation[s].x << " x " << sitesLocation[s].y;
    std::cout << ") : " << *actions[act[s]].action << " in "  << look4name(actions[act[s]].state) << " (" << val[s] << ")" << std::endl;
  } // for s in sites

  msg.policy = pol;
  msg.goal_name = *actions[TARGET].action + "( " + msg.final_state + " )";
  policy_pub.publish(msg);
} // plan()

void PRUplanner::chooseAction(int &bestA, float &bestV, std::vector<struct act> &actions, std::vector<float> &val, std::vector<float> &sval, std::vector<struct state> &states, std::vector<struct sstate> &specialStates, int s) {
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
	expO = val[itRes->s];   // value of a fixed site
	nloc = states[itRes->s].location;
      } else {
	expO = sval[-1-itRes->s]; // value of a special state
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
void PRUplanner::planStochastic() {
  int closestSite;
  if (! updateDistances(closestSite))
    return; // unable to update distances

  std::vector<struct state>  states;
  std::vector<struct sstate> specialStates;  

  std::vector<struct act> actions;
  std::vector<struct act> specialActions; // a struct act for each special state

  std::vector<float> val[2];
  std::vector<float> sval[2];

  for (int j=0; j<fixedSites.size(); j++) {
    // go to site j
    std::string site = look4name(j);
    struct state st = {j, "location( "+site+" )"};
    states.push_back(st);
    for (std::map<std::string,float>::iterator it=siteActionReward[j].begin(); 
	 it != siteActionReward[j].end(); ++it) {
      // and act *it
      struct act cur;
      cur.action = &it->first;
      cur.duration = siteActionDuration[j][it->first];
      if (*cur.action == GOAL_ADVERTISE_COMPLEX) {
	std::string *act = &siteActionParam[j][it->first];
	struct sstate st2 = {j, "special_location( "+site+", "+*act+" )",NULL};
	struct outcome res1 = {0.5, j, it->second};
	struct outcome res2 = {0.5, -1-specialStates.size(), it->second};
	specialStates.push_back(st2);
	sval[0].push_back(0); // value 0
	sval[1].push_back(0); // value 0
	cur.outcomes.push_back(res2);	
	cur.outcomes.push_back(res1);
	struct act spec;
	spec.action = act;
	struct outcome res = {1.0, j, SPEC_REWARD};
	spec.outcomes.push_back(res);
	spec.duration = SPEC_DURATION;
	specialActions.push_back(spec);
      } else if (*cur.action == GOAL_INTERACT) {
	struct sstate st2 = {j, "special_location( "+site+", goal )",&skipStr};
	struct outcome res = {1.0, -1-specialStates.size(), it->second};
	specialStates.push_back(st2);
	sval[0].push_back(0); // value 0
	sval[1].push_back(0); // value 0
	cur.outcomes.push_back(res);
	struct act spec;
	spec.action = &skipStr;
	struct outcome res_ = {1.0, -specialStates.size(), SKIP_REWARD};
	spec.outcomes.push_back(res_);
	spec.duration = 0;
	specialActions.push_back(spec);
      } else {
	struct outcome res = {1.0, 1+j, it->second};
	cur.outcomes.push_back(res);
      }
      actions.push_back(cur);
    } // for it
    val[0].push_back(0); // value 0
    val[1].push_back(0); // value 0
  } // for j

  if (actions.size() <= 0)
    return; // no action to plan yet

  struct state st = {-1, "location( RobotPos )"};
  states.push_back(st);
  val[0].push_back(0); // value 0
  val[1].push_back(0); // value 0

  int horizon = 10; // 10 actions long ?

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

  std::vector<std::vector<int> > act;
  std::vector<std::vector<int> > sact;

  for (int h=horizon-1; h>=0; --h) {
    int s;
#ifdef DEBUG_VI
    std::cout << "H"<<h<<":";
#endif
    act.push_back(std::vector<int>());
    sact.push_back(std::vector<int>());
    // updates the value of fixed states
    for (s=0; s<=fixedSites.size(); ++s) {
      float bestV = -1000000;
      int bestA = 0;
      chooseAction(bestA,bestV,actions,val[1-h%2],sval[1-h%2],states,specialStates,s);
      val[h%2][s] = bestV;
      act.back().push_back(bestA);
#ifdef DEBUG_VI
      std::cout << " "<<bestA<<"("<<bestV<<")";
#endif
    } // for state s
#ifdef DEBUG_VI
    std::cout << " +";
#endif
    // updates the value of special states
    for (s=0; s<specialStates.size(); ++s) {
      float bestV = -1000000;
      int bestA = 0;
      actions.push_back(specialActions[s]);
      chooseAction(bestA,bestV,actions,val[1-h%2],sval[1-h%2],states,specialStates,-1-s);
      actions.pop_back();
      if (bestA>=actions.size())
	bestA = -1 - s;
      sval[h%2][s] = bestV;
      sact.back().push_back(bestA);
#ifdef DEBUG_VI
      std::cout << " "<<bestA<<"("<<bestV<<")";
#endif
    } // for s
#ifdef DEBUG_VI
    std::cout << std::endl;
#endif
  } // for h

  t41_robust_navigation::Policy msg;
  float bestReward = 0;
  int TARGETaction = 0;
  std::set<int> reachableStates;
  int TARGETstate = look4target(actions,specialActions,val[0],sval[0],act,sact, states, specialStates, reachableStates, fixedSites.size(), horizon-1, 0, 0, bestReward, TARGETaction);

  if (TARGETstate<0) {
    msg.final_state = specialStates[-1-TARGETstate].name;
  } else {
    msg.final_state = states[TARGETstate].name;
  }
  std::string *initialState;

  std::vector<t41_robust_navigation::StatePolicy> pol;
  std::cout << "Optimal plan is \n";
  for (int steps=0; steps<horizon; ++steps) {
  for (int s=0; s<states.size(); ++s) {
    t41_robust_navigation::StatePolicy stateAction;
    stateAction.state = str(boost::format("location( %s, %d )") %look4name(states[s].location) %steps); //"location( "+look4name(states[s].location)+", "+stepsStr+" )";
    initialState = & stateAction.state; // The robot position is always the last one
    struct act *a;
    if (act[steps][s] >= 0)
      a = &actions[act[steps][s]];
    else
      a = &specialActions[-1-act[steps][s]];
      
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
	stateAction.successors.push_back(str(boost::format("location( %s, %d )") %look4name(specialStates[-1-itRes->s].location) %(steps+1))); //"location( "+look4name(specialStates[-1-itRes->s].location)+", "+stepsStr1+" )");
      else
	stateAction.successors.push_back(str(boost::format("location( %s, %d )") %look4name(states[itRes->s].location) %(steps+1)));// "location( "+look4name(states[itRes->s].location+", "+stepsStr1+" )");
    } // for *itRes in successors of a
    pol.push_back(stateAction);
    std::cout << s << " - " << stateAction.state ;
    std::cout << ": " << stateAction.action << " (" << val[0][s] << ")" << std::endl;
  } // for s in sites
  for (int s=0; s<specialStates.size(); s++) {
    t41_robust_navigation::StatePolicy stateAction;
    std::vector<std::string> elts;
    boost::split(elts, specialStates[s].name, boost::is_any_of("( ,)"), boost::token_compress_on);
    stateAction.state = str(boost::format("special_location( %s, %d, %s )") %elts[1] %steps %elts[2]);//specialStates[s].name;
    struct act *a;
    if (sact[steps][s] >= 0)
      a = &actions[sact[steps][s]];
    else
      a = &specialActions[-1-sact[steps][s]];

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
	stateAction.successors.push_back(str(boost::format("location( %s, %d )") %look4name(specialStates[-1-itRes->s].location) %(steps+1)));//look4name(specialStates[-1-itRes->s].location+", "+stepsStr1+" )");
      else
	stateAction.successors.push_back(str(boost::format("location( %s, %d )") %look4name(states[itRes->s].location) %(steps+1)));//look4name(states[itRes->s].location+", "+stepsStr1+" )");
    } // for *itRes in successors of a
    pol.push_back(stateAction);
    std::cout << (-1-s) << " - " << stateAction.state ;
    std::cout << ": " << stateAction.action << " (" << sval[0][s] << ")" << std::endl;
  } // for s in sites
  } // for horizon h

  msg.policy = pol;
  msg.initial_state = *initialState;
  if (TARGETaction<0)
    msg.goal_name = *specialActions[-1-TARGETaction].action;
  else
    msg.goal_name = *actions[TARGETaction].action;
  if (TARGETstate<0)
    msg.goal_name +=  "( " + look4name(specialStates[-1-TARGETstate].location) + " )";
  else
    msg.goal_name +=  "( " + look4name(states[TARGETstate].location) + " )";
  policy_pub.publish(msg);

} // planStochastic()

int PRUplanner::look4target(std::vector<struct act>& actions, std::vector<struct act>& specialActions, 
			    std::vector<float>& values, std::vector<float>& specialValues, 
			    std::vector<std::vector<int> >& policy, std::vector<std::vector<int> >& specialPolicy,
			    std::vector<struct state>& states, std::vector<struct sstate>& specialStates,
			    std::set<int> &reachableStates, int startState, int stepsLeft,
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
      bestState = look4target(actions,specialActions,values,specialValues,policy,specialPolicy, states, specialStates, reachableStates, it->s, stepsLeft-1, 
			      accReward+it->r*it->p, action, bestReward, bestAction);
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
