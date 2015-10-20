#include "T42.h"
#include "t41_robust_navigation/policyResults.h"
#include "t41_robust_navigation/PolicyResult.h"
#include "t41_robust_navigation/Policy.h"
#include "t11_kb_modeling/GetAllSites.h"
#include "PRUplus.h"
#include "PRU2MDP.h"
#include "MDP.h"

#include <boost/format.hpp>

// <node pkg="t42_multi_robot_planning" type="pruPlanner2" name="PRU_planner" respawn="false" output="screen" launch-prefix="valgrind --db-attach=yes">

#define DEBUG_VI

//#define PATIENCE 0.99
//#define HORIZON 10

//#define ROBOT_NAME std::string("diago")

static float testFunction(const PRUstate& fromState, const PRUstate& toState,
       const string& kind, float parameter) {
  return 0;
}

class PRUplanner : public T42 {
private:
  // ROS part
  ros::Publisher policy_pub;
  ros::Subscriber result_sub;
  void resultCallback(const t41_robust_navigation::PolicyResult::ConstPtr& msg);

  // parameters
  std::string ROBOT_NAME, pruFolder, pruFile;
  int HORIZON;
  double PATIENCE;

  PRUplus *pru;
  MDP *mdp;

  void prepareModel();

public:
  PRUplanner (ros::NodeHandle node);
  virtual void noMoveCallBack();
  virtual void plan();
};

PRUplanner::PRUplanner(ros::NodeHandle node) : T42(node) {
  ros::NodeHandle lnode("~");
  lnode.param("pru_folder",pruFolder,std::string("."));
  lnode.param("patience",PATIENCE,0.99);
  lnode.param("horizon",HORIZON,10);
  lnode.param("robot_name",ROBOT_NAME, std::string("diago"));
  lnode.param("PRU",pruFile,std::string("pruDiag.xml"));
  if (*pruFolder.rbegin() != '/')
    pruFolder += '/';
  if (*pruFile.begin() == '/')
    pruFile.erase(pruFile.begin(),pruFile.begin());

  policy_pub = node.advertise<t41_robust_navigation::Policy>(TOPIC_POLICY, 100);
  result_sub = node.subscribe(TOPIC_POLICY_RESULT, 10, &PRUplanner::resultCallback, this);

  // Reads the PRU
  //static PRUplus pru2 (pruFolder+pruFile);
  //PRUplus *pru2 = new PRUplus(pruFolder+pruFile);
  PRUplus::domainFunction = &testFunction;
  pru = new PRUplus();
  pru->readXML(pruFolder+pruFile);
  ROS_INFO("PRU %s%s has been read.",pruFolder.c_str(),pruFile.c_str());
  std::cout << *pru;

  ros::ServiceClient service_get_all_sites = node.serviceClient<t11_kb_modeling::GetAllSites>("get_all_sites");
  t11_kb_modeling::GetAllSites srv;
  if (! service_get_all_sites.call(srv)) {
    ROS_WARN("Unable to get sites");
    exit(1);
  }
  std::vector<std::string>::const_iterator it = srv.response.ids.begin();
  while (it != srv.response.ids.end()) {
    std::cerr << "Registering fixed-site " << *it << std::endl;
    std::map<std::string, int>::iterator locIt = fixedSites.find(std::string(*it));
    int siteID = -1;
    if (locIt == fixedSites.end())
      siteID = newFixedSite(*it);
    else
      siteID = locIt->second;
    if (siteID != -1) { // No goal at this place
      siteActionReward[siteID][""] = -1000;
      siteActionParam[siteID][""] = "";
      siteActionDuration[siteID][""] = 60;
    }
    ++it;
  }
  plan();
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
  MDPstate* goalState = NULL;
  for (int steps=0; steps<HORIZON; steps++) {
    bool firstStep = (steps==HORIZON-1);
    mdp->iterate(PATIENCE);
    const vector<const MDPaction*> &policy = mdp->getPolicy(steps);

    std::string cstep = str(boost::format(" & _step( %d )") %steps);
    for (int s=(firstStep?0:1); s<mdp->getStates().size(); ++s) { // put Init state only on first step
      t41_robust_navigation::StatePolicy stateAction;
      if ((mdp->getState(s) == goalState) && !firstStep)
        continue;
      stateAction.state = mdp->getState(s)->getPredicates(cstep);
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
	t41_robust_navigation::StateOutcome so;
	so.observation = (*it)->prevOutcome->observable;
	so.successor = (*it)->getPredicates(ostep);
        stateAction.outcomes.push_back(so);
        if ((*it)->prevOutcome->isFinal)
          goalState = *it;
      } // for it in outcomes of a
      pol.push_back(stateAction);
      std::cout << s << " - " << stateAction.state ;
      std::cout << ": " << stateAction.action << std::endl;
    } //for s
    ostep = cstep;
  } // for int steps

  msg.policy = pol;

  msg.initial_state = mdp->getState(0)->getPredicates();
  if (goalState != NULL)
    msg.final_state = goalState->getPredicates();
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

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pruPlanner");

  ros::NodeHandle node;

  PRUplanner t42(node);
  t42.run();

  return 0;
}
