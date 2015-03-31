// Compile a PRU+ into a MDP
#ifndef __PRU2MDP__
#define __PRU2MDP__

#include "PRUplus.h"

class MDPaction;

class PRU2MDPprogress {
 private:
  void buildActions(const PRUmodule *mod, 
		    map<string,domain_type>::const_iterator iParam,
		    const MDPaction &act);
 public:
  const PRUlayer *lay;
  const map<string, const string*> stateVariables;
  vector<MDPaction*> actions; // actions are created but not deleted here

  PRU2MDPprogress(const PRUlayer *l, const map<string, const string*> &sv);
};

class PRU2MDP {
 private:
  void buildProgress(const PRUlayer *layer,
		     vector<string>::const_iterator itSV, 
		     map<string, const string*> &params);
 public:
  vector<PRU2MDPprogress*> progress;
  map<string, domain_type> *stateVariableDomain;

  PRU2MDP (const PRUplus &pru);
};

#endif
