// Stores a PRU+ file in memory for later use
#include <string>
#include <vector>
#include <map>
#include <libxml++/libxml++.h>
#include <libxml++/parsers/textreader.h>

#include <iostream>

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

  PRUoutcome() { initDefaultValues(); };
  PRUoutcome(xmlpp::TextReader &reader);
  ~PRUoutcome() { std::cout << "Destroying outcome " << name <<std::endl;};
};

class PRUmodule {
 public:
  string actionName;
  map<string, domain_type > parameters;
  vector<PRUoutcome*> outcomes;
  PRUmodule() { };
  PRUmodule(xmlpp::TextReader &reader);
  ~PRUmodule() {
    std::cout << "Destroying Module "<<actionName <<std::endl;
    for (vector<PRUoutcome*>::iterator it = outcomes.begin(); it != outcomes.end(); ++it)
      delete *it;
  };
};

class PRUlayer {
 public:
  string name;
  vector<string> stateVariables;
  vector<PRUmodule*> modules;
  PRUlayer() { };
  PRUlayer(xmlpp::TextReader &reader);
  ~PRUlayer() {
    std::cout << "Destroying Layer " << name <<std::endl;
    for (vector<PRUmodule*>::iterator it = modules.begin(); it != modules.end(); ++it)
      delete *it;
  };
};

class PRUplus {
 private:
  void readXML(xmlpp::TextReader &reader);
 public:
  vector<string> stateVariablesInitialAssignments;
  vector<string> firstEnabledModules;
  vector<PRUlayer*> layers;
  PRUplus() { std::cout << "Empty \n" <<std::endl; };
  PRUplus(string xlmFileName);
  ~PRUplus() {
    std::cout << "Destroying PRU" <<std::endl;
    for (vector<PRUlayer*>::iterator it = layers.begin(); it != layers.end(); ++it)
      delete *it;
  };
};
