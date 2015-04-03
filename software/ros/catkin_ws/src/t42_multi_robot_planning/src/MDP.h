// Represents a MDP for solving PRUs
#ifndef __MDP__
#define __MDP__

#include "PRUplus.h"

class MDPstate;

/** Represents a MDP action.
 * A MDP action is a PRUaction with instanciated parameters that can be used in a specific state of a the layer the PRUaction belongs to.
 */
class MDPaction {
 public:
  const string &actionName;
  map<string, const string*> parameters; // reuse strings from actions domain (PRUmodule.parameters)
  set<MDPstate*> outcomes;

  MDPaction(const string &name);  
  MDPaction(const MDPaction &copy);

  /** Copies the specified action, changing the given parameter by the specified value.*/
  MDPaction(const MDPaction &copy, string param, const string *value);

  ~MDPaction();  
  
  string const* getParameter(string &key) const
;
  friend std::ostream& operator<<(std::ostream& os, const MDPaction& act); // allows that method to access private fields
 
};
std::ostream& operator<<(std::ostream& os, const MDPaction& act);

/** Represents a MDP state resulting from the outcome of an action (with specific parameters) in a layer (with specific variables). */
class MDPstate {
 public:
  const string name;
  set<const MDPaction *> availableActions; // references to PRU2MDPprogress.actions
  const MDPaction *prevAction; // may be null for initial state
  const PRUoutcome *prevOutcome; // may be null for initial state
  map<string, const string*> stateVariables; // references to PRU2MDP.stateVariableDomain

  MDPstate(const string &description);
  MDPstate(const string &description, const MDPaction *act, const PRUoutcome *out,
	   const map<string, const string*> &SV);

  ~MDPstate(); 
};

class MDP {
 private:
  map<string, MDPstate> states;
  map<string, MDPaction*> actions; // association of each action with its name
  vector<vector<float> > rewards; // a float for each action of each state
  vector<vector<float> > durations; // a float for each action of each state
  map<string, domain_type> *stateVariableDomain;

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
