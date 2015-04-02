// Compile a PRU+ into a MDP
#ifndef __PRU2MDP__
#define __PRU2MDP__

#include "PRUplus.h"
#include "MDP.h"
#include "PRU2MDPactionDescriptor.h"

class MDPaction;
class PRU2MDP;

/**
 * Represents a PRU layer and the associated state variables.
 * Builds and storess the actions defined for this set of variables.
 */
class PRU2MDPprogress {
 private:
  void buildActions(const PRUmodule *mod, 
		    map<string,domain_type>::const_iterator iParam,
		    const MDPaction &act, map<string, domain_type> *SVdomain);
 public:
  const PRUlayer *lay;
  const map<string, const string*> stateVariables; // reuse strings from SV domains (PRU2MDP.stateVariableDomain)
  vector<MDPaction*> actions; // actions are created but not deleted here

  PRU2MDPprogress(const PRUlayer *l, const map<string, const string*> &sv,
		  map<string, domain_type> *stateVariableDomain);

  ~PRU2MDPprogress();

  /** Tests whether this progress has compatible state variables.
   * Specified state variables may include irrelevant ones.
   */
  bool isMatching(const map<string, const string*> &stateVariables) const;

  /** Matches all PRU modules with MDPactions from all MDPstate */
  void matchActions(const PRU2MDP *model);
  /** Matches specified modules with MDPactions and store them into s->availableActions */
  void matchActions(const vector<string> &nextModules, MDPstate *s, const PRU2MDP *model);

  /** Stores a descriptor of all the actions of this progress into the specified log */
  void registerActions(map<string, PRU2MDPactionDescriptor> &allActions) const;
};
std::ostream& operator<<(std::ostream& os, const PRU2MDPprogress& act);

/**
 * Builds a MDP from a PRUplus.
 */
class PRU2MDP {
 private:
  /** Builds Progresses from the specified PRULayer.
   * Each progress then builds instanciated actions.
   * Each action the builds resulting states.
   */
  void buildProgress(const PRUlayer *layer,
		     vector<string>::const_iterator itSV, 
		     map<string, const string*> &params);
 public:
  vector<PRU2MDPprogress*> progress;
  map<string, domain_type> *stateVariableDomain; // domains are created but not deleted here
  map<string, PRU2MDPactionDescriptor> allActions;

  PRU2MDP (const PRUplus &pru);
  ~PRU2MDP() {
    std::cout << "Destroying PRU2MDP...\n";
    for (vector<PRU2MDPprogress*>::iterator it=progress.begin();
	 it != progress.end(); ++it) {
      delete (*it);
    }
    delete stateVariableDomain; // TEMPORARY
  }

};

#endif
