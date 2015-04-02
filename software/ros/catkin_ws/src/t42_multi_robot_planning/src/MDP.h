// Compile a PRU+ into a MDP
#ifndef __MDP__
#define __MDP__

#include "PRUplus.h"

class MDPstate;

/** Represents a MDP action.
 * A MDP action is a PRUaction with instanciated parameters that can be used in a specific state of a the layer the PRUaction belongs to.
 */
class MDPaction {
  const string &actionName;
 public:
  map<string, const string*> parameters; // reuse strings from actions domain (PRUmodule.parameters)
  set<MDPstate*> outcomes;
 MDPaction(const string &name): actionName(name) {};
 MDPaction(const MDPaction &copy): 
  actionName(copy.actionName), parameters(copy.parameters), outcomes(copy.outcomes) {};
  /** Copies the specified action, changing the given parameter by the specified value.*/
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
  friend std::ostream& operator<<(std::ostream& os, const MDPaction& act); // allows that method to access private fields
};
std::ostream& operator<<(std::ostream& os, const MDPaction& act);

class MDPstate {
 public:
  const string name;
  set<const MDPaction *> availableActions;
  const MDPaction *prevAction; // may be null for initial state
  const PRUoutcome *prevOutcome; // may be null for initial state
  map<string, const string*> stateVariables;

 MDPstate(const string &description) : name(description) { 
    prevAction = NULL;
    prevOutcome = NULL;
  }
 MDPstate(const string &description, const MDPaction *act, const PRUoutcome *out,
	  const map<string, const string*> &SV) : 
  name(description), stateVariables(SV) {
    prevAction = act;
    prevOutcome = out;
  }
};

class MDP {
 private:
  map<string, MDPstate> states;
  map<string, MDPaction*> actions; // association of each action with its name
  vector<vector<float> > rewards; // a float for each action of each state
  vector<vector<float> > durations; // a float for each action of each state
  map<string, domain_type> *stateVariableDomain;

  /** Build the outcome state for a given state-variable instantiation 
      and a given action's outcome */
  void buildOutcome(PRUlayer *lay, PRUmodule *mod, string suffix, MDPaction *act,
		    PRUoutcome *out,
		    map<string, const string*> &params);

  /** Builds all the state-variable instantiations before building states
      for a given action */
  void buildOutcomes(PRUlayer *lay, PRUmodule *mod, string suffix, MDPaction *act,
		     vector<string>::const_iterator itSV, 
		     map<string, const string*> &params);

  /** Initiates the building of states for a given action */
  void buildAction(PRUlayer *lay, PRUmodule *mod, string suffix, MDPaction *act);

  /**
     Builds all the actions for this layer.module, 
     instanciating each remaining action-parameter
  */
  void buildActions(PRUlayer *lay, PRUmodule *mod, 
		    map<string,domain_type>::const_iterator iParam,
		    string suffix, MDPaction *act);

  /**
     Looks for the right MDPactions and inserts them into the specified state.
     for example: 
     + 1.doIt : the module doIt from layer 1
     + 2.* : any module from layer 2
     + 1.doIt$X=Y : the module doIt from layer 1 with parameter X set to Y
     + 2.*-2.doIt : any module from layer 2 but the module doIt
  */
  void associate(const string &module, MDPstate &state);

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
