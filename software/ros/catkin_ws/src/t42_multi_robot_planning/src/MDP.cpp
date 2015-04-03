// Represents a MDP for solving PRUs

#include "MDP.h"
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#undef PRINT
#include "DEBUGprint.h"


MDPaction::MDPaction(const string &name): actionName(name) {
  // Nothing more to do
};
MDPaction::MDPaction(const MDPaction &copy): 
  actionName(copy.actionName), parameters(copy.parameters) {
  for (set<MDPstate*>::const_iterator it=copy.outcomes.begin();
       it != copy.outcomes.end(); ++it) {
    outcomes.insert(new MDPstate((*it)->name, this, (*it)->prevOutcome,(*it)->stateVariables));
  }
};

MDPaction::MDPaction(const MDPaction &copy, string param, const string *value): 
  actionName(copy.actionName), parameters(copy.parameters) {
    parameters[param] = value;
  for (set<MDPstate*>::const_iterator it=copy.outcomes.begin();
       it != copy.outcomes.end(); ++it) {
    outcomes.insert(new MDPstate((*it)->name, this, (*it)->prevOutcome,(*it)->stateVariables));
  }
};
string const* MDPaction::getParameter(string &key) const {
    map<string, const string*>::const_iterator it = parameters.find(key);
    if (it == parameters.end())
      return NULL;
    else
      return it->second;
  };
MDPaction::~MDPaction() {
  for (set<MDPstate*>::iterator it = outcomes.begin();
       it != outcomes.end(); ++it)
    delete (*it);
};

std::ostream& operator<<(std::ostream& os, const MDPaction& act) {
  os << act.actionName;
  for (map<string, const string*>::const_iterator it = act.parameters.begin();
	 it != act.parameters.end(); ++it) {
    os << ' ' << it->first<<"="<<*it->second;
  }
  return os;
};

MDPstate::MDPstate(const string &description) : name(description) { 
  prevAction = NULL;
  prevOutcome = NULL;
};
MDPstate::MDPstate(const string &description, const MDPaction *act, 
		   const PRUoutcome *out, const map<string, const string*> &SV) : 
  name(description), stateVariables(SV) {
  prevAction = act;
  prevOutcome = out;
};
MDPstate::~MDPstate() {
#ifdef PRINT
  std::cout << "Destroying state " << name;
  if (prevAction != NULL)
    std::cout << ".(action " << prevAction->actionName
	      << " -> " << prevOutcome->name << ")";
  std::cout << std::endl;
#endif
};
