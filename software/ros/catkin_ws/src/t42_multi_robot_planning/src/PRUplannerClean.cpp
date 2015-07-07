#include "T42.h"
#include "t41_robust_navigation/policyResults.h"
#include "t41_robust_navigation/PolicyResult.h"
#include "t41_robust_navigation/Policy.h"
#include "PRUplus.h"
#include "PRU2MDP.h"
#include "MDP.h"


#include <boost/format.hpp>

#define DEBUG_VI

#define PATIENCE 0.99
#define HORIZON 10

#define ROBOT_NAME std::string("diago")

class PRUplanner : public T42 {
private:
  // ROS part
  ros::Publisher policy_pub;
  ros::Subscriber result_sub;
  void resultCallback(const t41_robust_navigation::PolicyResult::ConstPtr& msg);

  PRUplus *pru;
  MDP *mdp;

  void prepareModel();

public:
  PRUplanner (ros::NodeHandle node);
  virtual void noMoveCallBack();
  virtual void plan();
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

  PRU2MDP p2m(*pru);
  mdp = p2m.getMDP();
  mdp->initVI();

  std::cout << "Optimal plan is \n";
  t41_robust_navigation::Policy msg;
  std::string ostep = " & _step( 0 )";
  std::vector<t41_robust_navigation::StatePolicy> pol;
  for (int steps=0; steps<HORIZON; steps++) {
    mdp->iterate(PATIENCE);
    const vector<const MDPaction*> policy;

    std::string cstep = str(boost::format(" & _step( %d )") %steps);
    for (int s=0; s<mdp->getStates().size(); ++s) {
      t41_robust_navigation::StatePolicy stateAction;
      stateAction.state = mdp->getState(s)->getPredicates() + cstep;
      const MDPaction *a = policy[s];
      stateAction.action = a->actionName;
      if (! a->parameters.empty()) {
        stateAction.action += "( ";
        bool notFirst = false;
        for (PRUstate::const_iterator it = a->parameters.begin();it != a->parameters.end(); ++it) {
          if (notFirst)
            stateAction.action += ", ";
          else
            notFirst = true;
          stateAction.action += *(it->second);
        } // for it in a's parameters
        stateAction.action += " )";
      } // if parametric action
      for (std::set<MDPstate*>::const_iterator it = a->outcomes.begin(); it!= a->outcomes.end(); ++it) {
        stateAction.successors.push_back((*it)->getPredicates() + ostep);
      } // for it in outcomes of a
      pol.push_back(stateAction);
      std::cout << s << " - " << stateAction.state ;
      std::cout << ": " << stateAction.action << std::endl;
      ostep = cstep;
    } //for s
  } // for int steps

  msg.policy = pol;

  msg.initial_state = str(boost::format("%s & _step( %d )") %mdp->getState(0)->getPredicates() %(HORIZON-1));
/*
  if (TARGETaction<0)
    msg.goal_name = *specialActions[-1-TARGETaction].action;
  else
    msg.goal_name = *actions[TARGETaction].action;
  if (TARGETstate<0)
    msg.goal_name +=  "( " + look4name(specialStates[-1-TARGETstate].location) + " )";
  else
    msg.goal_name +=  "( " + look4name(states[TARGETstate].location) + " )";
*/
  msg.goal_name = "printer_assist";
  policy_pub.publish(msg);
}

void PRUplanner::prepareModel() {
  // First reads the PRU
  pru = new PRUplus("pruDIAG.xml");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pruPlanner");

  ros::NodeHandle node;

  PRUplanner t42(node);
  t42.run();

  return 0;
}
