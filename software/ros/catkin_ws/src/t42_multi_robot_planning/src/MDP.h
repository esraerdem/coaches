// Compile a PRU+ into a MDP
#ifndef __MDP__
#define __MDP__

#include "PRUplus.h"

class MDPstate;

class MDPaction {
  const string &actionName;
public:
  map<string, const string*> parameters;
  set<MDPstate*> outcomes;
  MDPaction(const string &name): actionName(name) {};
  MDPaction(const MDPaction &copy): 
    actionName(copy.actionName), parameters(copy.parameters), outcomes(copy.outcomes) {};
  MDPaction(const MDPaction &copy, string param, const string *value): 
    actionName(copy.actionName), parameters(copy.parameters), outcomes(copy.outcomes) {
    parameters[param] = value;
  };
  string const* getParameter(string &key) {
    map<string, const string*>::const_iterator it = parameters.find(key);
    if (it == parameters.end())
      return NULL;
    else
      return it->second;
  };
    friend std::ostream& operator<<(std::ostream& os, const MDPaction& act); // allows this method to access private fields
};
std::ostream& operator<<(std::ostream& os, const MDPaction& act);

class MDPstate {
public:
  const string name;
  set<const MDPaction *> availableActions;
  MDPaction *prevAction; // may be null for initial state
  PRUoutcome *prevOutcome; // may be null for initial state
  map<string, const string*> stateVariables;

 MDPstate(const string &description) : name(description) { }
};

class MDP {
private:
  map<string, MDPstate> states;
  map<string, MDPaction*> actions; // association of each action with its name
  vector<vector<float> > rewards; // a float for each action of each state
  vector<vector<float> > durations; // a float for each action of each state
  map<string, domain_type> *stateVariableDomain;

  void buildOutcomes(PRUlayer *lay, PRUmodule *mod, string suffix, MDPaction *act,
		     vector<string>::const_iterator itSV, 
		     map<string, const string*> &params);

  void buildAction(PRUlayer *lay, PRUmodule *mod, string suffix, MDPaction *act);

  /**
     Builds all the actions for this layer.module, 
     instanciating each remaining parameter
   */
  void buildActions(PRUlayer *lay, PRUmodule *mod, 
		    map<string,domain_type>::const_iterator iParam,
		    string suffix, MDPaction *act);

public:
  ~MDP() {
    for (map<string, MDPaction*>::iterator it=actions.begin();
	 it != actions.end(); ++it) {
      delete (it->second);
    }
  }

  MDP (PRUplus &pru, map<string, domain_type> *stateVariableDomain);
}; // class MDP

#endif
