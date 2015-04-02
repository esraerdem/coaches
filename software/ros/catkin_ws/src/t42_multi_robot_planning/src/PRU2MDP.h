// Compile a PRU+ into a MDP
#ifndef __PRU2MDP__
#define __PRU2MDP__

#include "PRUplus.h"

class MDPaction;

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
};
std::ostream& operator<<(std::ostream& os, const PRU2MDPprogress& act);

/**
 * Builds a MDP from a PRUplus.
 */
class PRU2MDP {
 private:
  void buildProgress(const PRUlayer *layer,
		     vector<string>::const_iterator itSV, 
		     map<string, const string*> &params);
 public:
  vector<PRU2MDPprogress*> progress;
  map<string, domain_type> *stateVariableDomain; // domains are created but not deleted here

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
