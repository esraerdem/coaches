// Stores a PRU+ file in memory for later use
#include <string>
#include <vector>
#include <map>

using std::string;
using std::map;
using std::vector;

typedef vector<string> domain_type;

class PRUstate;
class PRUmodule;

typedef float (*function)(const PRUstate& fromState, 
			  const PRUmodule& withModule, 
			  const PRUstate& toState, 
			  const string& kind,
			  float parameter);
class PRUoutcome {
 public:
  string name;
  float probability;
  
  string observable; /** Logic formula true iff this outcome is valid */

  string quality;
  float qualityParameter;
  float qualityConstant;

  string duration;
  float durationParameter;
  float durationConstant;
  
  vector<string> stateVariableUpdate;
  vector<string> nextModules;
  
  bool isFinal;
  string finalLabel; /** empty or the label of the outcome of this PRU+ */

  PRUoutcome() {
    // set default value
    probability=1.0f;
    quality="null"; 
    qualityParameter=0;
    qualityConstant=0;
    duration="null";
    durationParameter=0; 
    durationConstant=0; 
    isFinal = false;
    finalLabel = "";
  }
};

class PRUmodule {
 public:
  string actionName;
  map<string, domain_type > parameters;
  vector<PRUoutcome*> outcomes;
};

class PRUlayer {
 public:
  string name;
  vector<string> stateVariables;
  vector<PRUmodule*> modules;
};

class PRUplus {
 public:
  vector<string> stateVariablesInitialAssignments;
  vector<string> firstEnabledModules;
  vector<PRUlayer*> layers;
};
