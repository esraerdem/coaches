// Compile a PRU+ into a MDP
#include "PRUplus.h"

class MDPoutcome {
  //TODO
};

class MDPaction {
  const string &actionName;
  map<string, const string*> parameters;
  vector<MDPoutcome*> outcomes;
public:
  ~MDPaction() {
    for (vector<MDPoutcome*>::iterator it=outcomes.begin(); it!=outcomes.end(); ++it) {
      delete(*it);
    }
  }
  MDPaction(const string &name): actionName(name) {};
  MDPaction(const MDPaction &copy): 
    actionName(copy.actionName), parameters(copy.parameters), outcomes(copy.outcomes) {};
  MDPaction(const MDPaction &copy, string param, const string *value): 
    actionName(copy.actionName), parameters(copy.parameters), outcomes(copy.outcomes) {
    parameters[param] = value;
  };
};

class MDP {
private:
  vector<string> states;
  map<string, MDPaction*> actions; // association of each action with its name
  vector<vector<MDPaction*> > stateActions; // a vector of actions for each state
  vector<vector<float> > rewards; // a float for each action of each state
  vector<vector<float> > durations; // a float for each action of each state

  void buildAction(PRUlayer *lay, PRUmodule *mod, string suffix, MDPaction *act) {
    string id = lay->name + "." + mod->actionName + suffix;
    actions[id] = act;
    // TODO add outcomes and build associated states
    // remember to instanciate *lay's states variables
  }

  /**
     Builds all the actions for this layer.module, 
     instanciating each remaining parameter
   */
  void buildActions(PRUlayer *lay, PRUmodule *mod, 
		    map<string,domain_type>::const_iterator iParam,
		    string suffix, MDPaction *act) {
    if (iParam == mod->parameters.end())
      buildAction(lay,mod,suffix,act);
    else {
      string name = iParam->first;
      domain_type::const_iterator it = iParam->second.begin();
      domain_type::const_iterator itEnd = iParam->second.end();
      ++iParam;
      for (;it != itEnd; ++it) {
	buildActions(lay, mod, iParam, suffix+","+name+"="+*it, 
		     new MDPaction(*act,name,&*it));
      } // for it in the current parameter's domain
      --iParam;
    } // if more parameters
  } // buildActions(...)

public:
  ~MDP() {
    for (map<string, MDPaction*>::iterator it=actions.begin();
	 it != actions.end(); ++it) {
      delete (it->second);
    }
  }

  MDP (PRUplus &pru) {
    states.push_back("robotPos");
    // TODO get transitions for initial state
    for (vector<PRUlayer*>::const_iterator iLay = pru.layers.begin(); 
	 iLay != pru.layers.end(); ++iLay) {
      PRUlayer *lay = *iLay;
      stateActions.push_back(vector<MDPaction*>());
      for (vector<PRUmodule*>::const_iterator iMod = lay->modules.begin(); 
	   iMod != lay->modules.end(); ++iMod) {
	PRUmodule *mod = *iMod;
	if (mod->parameters.empty())
	  buildAction(lay, mod, "", new MDPaction(mod->actionName));
	else
	  buildActions(lay, mod, mod->parameters.begin(), "", 
		       new MDPaction(mod->actionName));
      } // for iMod in lay->modules
    } // for iLay in pru.layers
  }
}; // class MDP
