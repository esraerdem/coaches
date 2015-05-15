#include "T42.h"
#include "t41_robust_navigation/policyResults.h"
#include "t41_robust_navigation/PolicyResult.h"
#include "t41_robust_navigation/Policy.h"

#include <boost/format.hpp>

#define DEBUG_VI

#define MAX_PLAN_LENGTH 10
#define GOAL_REWARD 20000
#define PATIENCE 0.99
#define SPEC_REWARD 200
#define SPEC_DURATION 1
#define SKIP_REWARD 80

#define ROBOT_NAME std::string("diago")

T42 *app = NULL;

static float domainFunction(const PRUstate& fromState, const PRUstate& toState,
		   const string& kind, float parameter) {
  if (kind == "null")
    return 0;
  if (kind == "distance") {
    PRUstate::const_iterator itF = fromState.find("location");
    PRUstate::const_iterator itT = toState.find("location");
    if ((itF==fromState.end()) || (itT==toState.end())) {
      std::cerr << "Unable to compute distance, locations not specified" << std::endl;
      return 0;
    }
    return app->getDistance(itF->second, itT->second); // gets the distance from the movebase cache
  }
  std::cerr << "Unknown function kind " << kind << "!" << std::endl;
  return 0;
}

class Planner : public T42 {
private:
  // ROS part
  ros::Publisher policy_pub;
  ros::Subscriber result_sub;

  void resultCallback(const t41_robust_navigation::PolicyResult::ConstPtr& msg);

  // Planning part
  PRUplus *pru;        // the PRU to instanciate
  PRU2MDP *translator; // the PRU to MDP translator - alive only from translation to plan publication
  MDP     *mdp;        // the resulting MDP - alive only from translation to plan publication
  void prepareModel();
  int computePlan(int horizon);
  void look4target(int startState, int stepsLeft, int &bestState,
		  float accReward, int lastAction, float &bestReward, int &bestAction);
  bool publishPlan(int horizon); // returns true if plan have been sent

  std::set<std::pair<std::string,int> > actionUsed; // a set of <action, location> to track if some action has already been used at some location

public:
  Planner (ros::NodeHandle node);
  virtual void noMoveCallBack();
  virtual void plan();
  void planStochastic();
};

Planner::Planner(ros::NodeHandle node) : T42(node) {
  policy_pub = node.advertise<t41_robust_navigation::Policy>(TOPIC_POLICY, 100);
  result_sub = node.subscribe(TOPIC_POLICY_RESULT, 10, &Planner::resultCallback, this);

  // Read the PRU
  std::locale::global(std::locale(""));
  setlocale(LC_NUMERIC,"C");
  PRUplus::domainFunction = &domainFunction;
  pru = new PRUplus("pru.xml");
}

void Planner::resultCallback(const t41_robust_navigation::PolicyResult::ConstPtr& msg)
{
  if (msg->feedback != POLICY_SUCCESS) {
    // TODO update taboo and replan
  } // if policy has failed
  // replan only if KB ask to
}

void Planner::noMoveCallBack() {
  //plan(); // plan only if KB ask to
}

void Planner::plan() {
  int closestSite;
  if (! updateDistances(closestSite))
    return; // unable to update distances
  
  prepareModel();

  if (mdp == NULL)
    return; // no action to plan yet

  int horizon = computePlan(MAX_PLAN_LENGTH); // max 10 actions long ?

  //publishPlan(horizon); // plan was already published if ok

  delete(mdp); mdp = NULL; 
}

void Planner::prepareModel() {
  translator = new PRU2MDP(*pru);
  mdp = translator->getMDP();
}

int Planner::computePlan(int horizon) {
  mdp->initVI();
  for (int h=0; h<horizon; ++h) {
    int s;
#ifdef DEBUG_VI
    std::cout << "H"<<h<<":";
#endif
    mdp->iterate(patience);

    if (publishPlan(h))
      return h+1;
  } // for h
  return horizon;
} // computePlan(h)

/*
void Planner::look4target(MDPstate const *startState, int stepsLeft, MDPstate const * &bestState,
			  float accReward, MDPaction const *lastAction, float &bestReward, MDPaction const * &bestAction) {
 */
bool Planner::publishPlan(int horizon) {
  t41_robust_navigation::Policy msg;
  float bestReward = -1;
  MDPaction const *TARGETaction = NULL;
  actionUsed.clear();
  MDPstate const *TARGETstate = NULL;
  look4target(mdp->getState(0), horizon, TARGETstate,0, NULL, bestReward, TARGETaction);
  if (bestReward < 0) {
    if (horizon<=0) {
      std::cout << "Plan failed, no previous one" << std::endl;
      return true;
    }
    std::cout << "Plan failed, publishing previous one" << std::endl;
    look4target(fixedSites.size(), horizon, TARGETstate, 0, 0, bestReward, TARGETaction);
  } else {
    std::cout << "Plan is good, value=" << bestReward 
	      << " goal=" << TARGETaction->actionName 
	      << " in " << TARGETstate->getPredicates() << std::endl;
    return false;
  }

  /* Example of policy to build:
    state: location( diago, restaurant) & desire( unknown, swipe ) & _step( 0 )
    action: interact( _P_1 )
    successors: ['location( diago, _P_1) & _goal( ) & _step( 0 )']
    state: location( diago, _P_1) & _goal( ) & _step( 0 )
    action: wait( _P_1 )
    successors: ['location( diago, _P_1) & _goal( ) & _step( 0 )']
    state: location( diago, doorEast ) & _step( 1 )
    action: advertise( phone )
    successors: ['location( diago, phone ) & _step( 0 )']
    state: location( diago, restaurant) & desire( unknown, swipe ) & _step( 1 )
    action: swipe( restaurant )
    successors: ['location( diago, restaurant ) & _step( 0 )']

    state = layer.action.outcome.variables
   */

  msg.final_state = TARGETstate->getPredicates();

  std::vector<t41_robust_navigation::StatePolicy> pol;
  std::cout << "Optimal plan is \n";
  std::string ostep = " & _step( 0 )";
  for (int steps=0; steps<=horizon; ++steps) {
    std::string cstep = str(boost::format(" & _step( %d )") %steps);
    for (int s=0; s<states.size(); ++s) {
      t41_robust_navigation::StatePolicy stateAction;
      stateAction.state = mdp->getState(s)->getPredicates() + cstep;
      MDPaction *a = mdp->getPolicy(steps)[s];
      
      stateAction.action = a->actionName + "(" ;
      bool notFirst = false;
      for (PRUstate::const_iterator it = a->parameters->begin();
	   it != a->parameters->end(); ++a) {
	if (notFirst) {
	  stateAction.action += ',';
	  notFirst = true;
	}
	stateAction.action += *it->second;	
      }
      stateAction.action += ')';
      
      for (std::vector<MDPstate *>::const_iterator itRes=a->outcomes.begin();
	   itRes != a->outcomes.end(); ++itRes) {
	MDPstate *dest = *itRes;
	stateAction.successors.push_back(dest->getPredicates + ostep);
      } // for *itRes in successors of a
      pol.push_back(stateAction);
      std::cout << s << " - " << stateAction.state ;
      std::cout << ": " << stateAction.action << std::endl;
    } // for s in sites
    ostep = cstep;
  } // for horizon h

  msg.policy = pol;

  msg.initial_state = str(boost::format("%s & _step( %d )") %mdp->getStates.front().getPredicates() %(horizon-1)); // initial state is the first state by construction
  if (TARGETaction != NULL) {
    msg.goal_name = TARGETaction->actionName + "(" ;
    MDPaction *a = TARGETaction;
    bool notFirst = false;
    for (PRUstate::const_iterator it = a->parameters->begin();
	 it != a->parameters->end(); ++a) {
      if (notFirst) {
	msg.goal_name.action += ',';
	notFirst = true;
      }
      msg.goal_name.action += *it->second;	
    }
    msg.goal_name.action += ')';
  } // if TARGETaction != NULL

  policy_pub.publish(msg);
  return true;
} // publishPlan(h)

void Planner::look4target(MDPstate const *startState, int stepsLeft, MDPstate const * &bestState,
			  float accReward, MDPaction const *lastAction, float &bestReward, MDPaction const * &bestAction) {
  const MDPaction *act;

  if (stepsLeft<0) { // end of the policy
    if (accReward>bestReward) {
      bestReward = accReward;
      bestAction = lastAction;
      bestState = startState;
    }
    return;
  }

  act = mdp->getPolicy(stepsLeft)[startState.index];

  for (std::vector<MDPstate*>::const_iterator it=act->outcomes.begin();
       it != act->outcomes.end(); ++it) {

    //TODO : Check the same action is not used twice in the same place in a domain-independent way :((
    //       Idea : add a "unique(stateVariable(s))" constraint in the XML file describing the PRU
    int loc;
    //      loc = states[it->s].location;
    std::pair<std::string, int> memory = std::pair<std::string, int>(*act->action, loc);
    if (actionUsed.insert(memory).second) {
      // new location-action combination
      float q = (*it)->prevOutcome->getQuality(startState->stateVariables, (*it)->stateVariables);
      look4target(*it, stepsLeft-1, bestState, 
		  accReward + q*(*it)->prevOutcome->probability, 
		  act, bestReward, bestAction);
      actionUsed.erase(memory);
    } else {
      // we would use the same action in the same place, as before
    }
  } // for *it in action.outcomes
} // look4target


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pruPlanner2");

  ros::NodeHandle node;

  Planner t42(node);
  app = &t42;
  t42.run();

  return 0;
}
