// Stores a PRU+ file in memory for later use
#ifndef __PRUplus__
#define __PRUplus__

#include <string>
#include <vector>
#include <map>
#include <set>
#include <libxml++/libxml++.h>
#include <libxml++/parsers/textreader.h>

#include <iostream>

using std::string;
using std::map;
using std::vector;
using std::set;

typedef set<string> domain_type;

/** Utility function for printing a domain */
std::ostream& operator<<(std::ostream& os, const domain_type vec);
/** Utility function for printing a set or variables */
std::ostream& operator<<(std::ostream& os, const map<string, domain_type> vars);

class PRUstate;
class PRUmodule;

/* The PRU function profile
typedef float (*function)(const PRUstate& fromState,
			  const PRUmodule& withModule,
			  const PRUstate& toState,
			  const string& kind,
			  float parameter);
*/

/** Represents one of the outcomes of a PRUmodule.
 *  Any outcome is defined by a name that must be unique for a given module.
 */
class PRUoutcome {
 private:
  void initDefaultValues() {
    probability=1.0f;
    quality="null";
    qualityParameter=0;
    qualityConstant=0;
    duration="null";
    durationParameter=0;
    durationConstant=0;
    isFinal = false;
    finalLabel = "";
  };

 public:
  string name;
  /** The estimated probability of this outcome. 
   * Must sum to 100% among all the outcomes of any module */
  float probability;

  /** Logic formula true iff this outcome is valid */
  string observable; 

  /** A function descriptor allowing for the computation of the expected quality of this outcome */
  string quality;
  float qualityParameter;
  float qualityConstant;

  /** A function descriptor allowing for the computation of the expected duration of this outcome */
  string duration;
  float durationParameter;
  float durationConstant;

  /** A list of updates for state variables.
   *  Each entry should be in the form X=V
   */
  vector<string> stateVariableUpdate;
  /** A list of modules that can be used after this outcome.
   *  Each entry should be a full-word regular expression.
   *  For example, advertise will match 1.advertise and 2.advertise but not 1.advertiseComplex
   */
  vector<string> nextModules;

  /** true iff this outcome completes the PRU (dead-end) */
  bool isFinal;
  /** empty or the label of the outcome of this PRU+ */
  string finalLabel; 

  PRUoutcome() { initDefaultValues(); };
  PRUoutcome(xmlpp::TextReader &reader);
  ~PRUoutcome() { std::cout << "||+Destroying outcome " << name <<std::endl;};
  /** Fills the specified domain with values from the SVUs of this outcome.
   *  Needs the action-parameters domain in case SVU depends on them. */
  void fillSVdomain (map<string, domain_type> &stateVariableDomain,
		     const map<string, domain_type> &parameters) const;
};

std::ostream& operator<<(std::ostream& os, const PRUoutcome& option);

/** Represents a module of a PRUlayer.
 *  A module is characterized by an actionName that must be unique in its layer.
 */
class PRUmodule {
 public:
  /** The action name. Will be used as an output of the MDP. */
  string actionName;
  /** The action parameters. Allow for action templates.
   *  For exemple, goto(X) with X={a,b,c}
   */
  map<string, domain_type> parameters;
  /** The list of possible outcomes of this module */
  vector<PRUoutcome*> outcomes;

  PRUmodule() { };
  PRUmodule(xmlpp::TextReader &reader);
  ~PRUmodule() {
    std::cout << "|+Destroying Module "<<actionName <<std::endl;
    for (vector<PRUoutcome*>::iterator it = outcomes.begin(); it != outcomes.end(); ++it)
      delete *it;
  };
  /** Fills the specified domain with values from the SVUs of all the outcomes of this module. */
  void fillSVdomain (map<string, domain_type> &stateVariableDomain) const;
};

std::ostream& operator<<(std::ostream& os, const PRUmodule& module);

/** Represents a layer from a PRUplus.
 *  A layer is characterized by a name that must be unique in each PRU.
 */
class PRUlayer {
 public:
  string name;
  /** A list of state variables that must be remembered all through this layer. */
  vector<string> stateVariables;
  /** The list of PRUmodules that can be used here */
  vector<PRUmodule*> modules;

  PRUlayer() { };
  PRUlayer(xmlpp::TextReader &reader);
  ~PRUlayer() {
    std::cout << "+Destroying Layer " << name <<std::endl;
    for (vector<PRUmodule*>::iterator it = modules.begin(); it != modules.end(); ++it)
      delete *it;
  };
  /** Fills the specified domain with values from the SVUs of all the modules of this layer. */
  void fillSVdomain (map<string, domain_type> &stateVariableDomain) const;
};

std::ostream& operator<<(std::ostream& os, const PRUlayer& layer);

/** Represents a PRU+. */
class PRUplus {
 private:
  void readXML(xmlpp::TextReader &reader);
 public:
  /** Stores the domain of any action that can be specified outside of the XML file.*/
  static map<string, domain_type> actionsDomain;
  /** Initial values of state variables. May be forgotten depending on layers... */
  vector<string> stateVariablesInitialAssignments;
  /** Available modules for starting the PRU. Usually first_layer.*
   *  Each entry should be a full-word regular expression.
   *  For example, advertise will match 1.advertise and 2.advertise but not 1.advertiseComplex
   */
  vector<string> firstEnabledModules;
  vector<PRUlayer*> layers;

  PRUplus() { };
  PRUplus(string xlmFileName);
  ~PRUplus() {
    std::cout << "Destroying PRU" <<std::endl;
    for (vector<PRUlayer*>::iterator it = layers.begin(); it != layers.end(); ++it)
      delete *it;
  };
  /** Fills the specified domain with values from the SVUs of all the modules of all the layers of this PRU. */  
  void fillSVdomain (map<string, domain_type> &stateVariableDomain) const;
};


std::ostream& operator<<(std::ostream& os, const PRUplus& pru);

#endif
