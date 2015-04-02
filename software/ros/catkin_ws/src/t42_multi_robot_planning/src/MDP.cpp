// Compile a PRU+ into a MDP
#include "MDP.h"
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

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

void MDP::buildOutcome(PRUlayer *lay, PRUmodule *mod, string suffix, MDPaction *act,
		       PRUoutcome *out, map<string, const string*> &params) {
  map<string, const string*> res(params); // makes a local copy of SV that will be updated with out SVUs
  string state = lay->name + '.' + mod->actionName;
  for (map<string, const string*>::const_iterator it=act->parameters.begin();
       it != act->parameters.end(); ++it)
    state += '$' + it->first + '=' + *(it->second);
  state += '.' + out->name;
  for (vector<string>::const_iterator itSVU = out->stateVariableUpdate.begin();
       itSVU != out->stateVariableUpdate.end(); ++itSVU) {
    vector<string> vec;
    boost::algorithm::split(vec, *itSVU, boost::algorithm::is_any_of(":= "), 
			    boost::algorithm::token_compress_on );
    if (vec.size()!=2)
      std::cerr << "Unreadable SVU : " << *itSVU << std::endl;
    else {
      if (vec[1][0] == '$') {
	string p = vec[1].substr(1);
	string const *v = act->getParameter(p);
	if (v == NULL)
	  std::cerr << "Unknown action parameter "<<vec[1]<<std::endl;
	else
	  res[vec[0]] = v;
      } else {
	// res[vec[0]] = vec[1]; // but needs a pointer to this local string...
	domain_type &dom = (*stateVariableDomain)[vec[0]];
	for (domain_type::const_iterator it = dom.begin();
	     it != dom.end(); ++it) {
	  if (*it == vec[1]) {
	    res[vec[0]] = &(*it);
	    break;
	  }
	} // for *it in this SV domain
      } // if not an action parameter
    } // if correct SVU element
  } // for itSVU in *itSVO SV Updates
  for (map<string, const string*>::const_iterator it = res.begin();
       it != res.end(); ++it) {
    state += '$' + it->first + '=' + *it->second;
  }
  map<string, MDPstate>::iterator itS = states.find(state);
  if (itS == states.end()) {
    std::cout << "+state " << state << std::endl;
    itS = states.insert(std::pair<string,MDPstate>(state, MDPstate(state))).first;
    itS->second.prevAction = act;
    itS->second.prevOutcome = out;
    itS->second.stateVariables = res;
  } else {
    std::cout << "~state " << state << std::endl;
  }
  act->outcomes.insert(&(itS->second));
} //buildOutcome

void MDP::buildOutcomes(PRUlayer *lay, PRUmodule *mod, string suffix, MDPaction *act,
			vector<string>::const_iterator itSV, 
			map<string, const string*> &params) {
  if (itSV == lay->stateVariables.end()) {
    // All state variable have been instanciated.
    // act is the current action (including parameters) ; 
    // params describe the initial state
    // mod is the raw module (including outcomes)
    /*
    std::cout << "State " << lay->name << suffix << " action " << *act;
    std::cout << " -> \n";
    for (vector<PRUoutcome*>::const_iterator it = mod->outcomes.begin();
	 it != mod->outcomes.end(); ++it) {
      std::cout << **it;//" > " << it->name << "\n";
    }
    std::cout << std::endl;
    */

    // Now lets build resulting states for this initial state
    for (vector<PRUoutcome*>::const_iterator itO = mod->outcomes.begin();
	 itO != mod->outcomes.end(); ++itO) {
      buildOutcome(lay, mod, suffix, act, *itO, params);
    } // for itO in mod->outcomes    
  } else {
    // Some state variable is to be instanciated yet
    string var = *itSV;
    map<string, domain_type>::const_iterator itV = stateVariableDomain->find(var);
    if (itV == stateVariableDomain->end())
      std::cerr << "Unknown state variable " << var << std::endl;
    else {
      ++itSV;
      for (domain_type::const_iterator itD = itV->second.begin();
	   itD != itV->second.end(); ++itD) {
	params[var] = &*itD;
	buildOutcomes(lay,mod,suffix+"$"+var+"="+*itD,act,itSV,params);
      } // for itD in variable domain
      --itSV;
      params.erase(var);
    } // if known state variable
  } // if some state variable left
} // buildOutcomes

void MDP::buildAction(PRUlayer *lay, PRUmodule *mod, string suffix, MDPaction *act) {
  string id = lay->name + "." + mod->actionName + suffix;
  act = new MDPaction(*act);
  actions[id] = act;
  std::cout << "Building action " << id << std::endl;
  map<string,const string*> params;
  buildOutcomes(lay,mod,".", act, lay->stateVariables.begin(), params);
} // buildAction(...)

void MDP::buildActions(PRUlayer *lay, PRUmodule *mod, 
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
      MDPaction ma(*act,name,&*it);
      buildActions(lay, mod, iParam, suffix+","+name+"="+*it, &ma);
    } // for it in the current parameter's domain
    --iParam;
  } // if more parameters
} // buildActions(...)

MDP::MDP (PRUplus &pru, map<string, domain_type> *stateVariableDomain) {
  // First build state variables' domain
  if (stateVariableDomain == NULL) {
    stateVariableDomain = new map<string, domain_type>();
    pru.fillSVdomain(*stateVariableDomain);
  }
  this->stateVariableDomain = stateVariableDomain;

  // Next build states and actions
  states.insert( std::pair<string,MDPstate>("robotPos", MDPstate("robotPos")) );
  for (vector<PRUlayer*>::const_iterator iLay = pru.layers.begin(); 
       iLay != pru.layers.end(); ++iLay) {
    PRUlayer *lay = *iLay;
    for (vector<PRUmodule*>::const_iterator iMod = lay->modules.begin(); 
	 iMod != lay->modules.end(); ++iMod) {
      PRUmodule *mod = *iMod;
      MDPaction act(mod->actionName); // template for building all those actions
      if (mod->parameters.empty())
	buildAction(lay, mod, "", &act);
      else
	buildActions(lay, mod, mod->parameters.begin(), "", &act);
    } // for *iMod in lay->modules
  } // for *iLay in pru.layers

  // Finally associate actions with states
  for (map<string, MDPstate>::iterator itS = states.begin();
       itS != states.end(); ++itS) {
    MDPstate &state = itS->second;
    if (state.prevAction == NULL) {
      // Initial state robotPos
      for(vector<string>::const_iterator itM = pru.firstEnabledModules.begin();
	  itM != pru.firstEnabledModules.end(); ++itM) {
	associate(*itM, state);
      } // for itM in pru.firstEnabledModules
    } else {
      // Outcome state
      for(vector<string>::const_iterator itM = state.prevOutcome->nextModules.begin();
	  itM != state.prevOutcome->nextModules.end(); ++itM) {
	associate(*itM, state);
      } // for itM in state.prevOutcome->nextModules
    }
  } // for *itS in states
} // MDP(...)

void MDP::associate(const string &module, MDPstate &state) {
  boost::regex expression("(^|\b)"+module+"(,|$)"); // adds the word-boundary conditions
  if (module[0]=='-')
    expression = boost::regex("(^|\b)"+module.substr(1)+"(,|$)");
  std::cout << "Matching " << module << " for state " << state.name << std::endl;
  for (map<string, MDPaction*>::const_iterator it = actions.begin();
       it != actions.end(); ++it) {
    if (regex_search(it->first, expression)) {
      if (module[0]=='-') {
	std::cout << " < " << it->first << std::endl;
	state.availableActions.erase(it->second);
      } else {
	std::cout << " > " << it->first << std::endl;
	state.availableActions.insert(it->second);
      }
    }
  /*
  std::string::const_iterator start, end; 
  start = file.begin(); 
  end = file.end(); 
  boost::smatch what; 
  boost::match_flag_type flags = boost::match_default; 
  while(regex_search(start, end, what, expression, flags)) { 
      // what[0] contains the whole string 
      // what[5] contains the class name. 
      // what[6] contains the template specialisation if any. 
      // add class name and position to map: 

      // update search position: 
      start = what[0].second; 
      // update flags: 
      flags |= boost::match_prev_avail; 
      flags |= boost::match_not_bob; 
  } // while more matches
  */
  }
} // associate

