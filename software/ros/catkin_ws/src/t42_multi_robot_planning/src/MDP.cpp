// Compile a PRU+ into a MDP
#include "MDP.h"
#include <boost/algorithm/string.hpp>

std::ostream& operator<<(std::ostream& os, const MDPaction& act) {
  os << act.actionName;
  for (map<string, const string*>::const_iterator it = act.parameters.begin();
	 it != act.parameters.end(); ++it) {
    os << ' ' << it->first<<"="<<*it->second;
  }
};

void MDP::buildOutcomes(PRUlayer *lay, PRUmodule *mod, string suffix, MDPaction *act,
			vector<string>::const_iterator itSV, 
			map<string, const string*> &params) {
  if (itSV == lay->stateVariables.end()) {
    // All state variable have been instanciated.
    // act is the current action (including parameters) ; 
    // params describe the initial state
    // mod is the raw module (including outcomes)
    std::cout << "State " << lay->name << suffix << " action " << *act;
    std::cout << " -> \n";
    for (vector<PRUoutcome*>::const_iterator it = mod->outcomes.begin();
	 it != mod->outcomes.end(); ++it) {
      std::cout << **it;//" > " << it->name << "\n";
    }
    std::cout << std::endl;
    // TODO Now lets build resulting states for this initial state
    for (vector<PRUoutcome*>::const_iterator itO = mod->outcomes.begin();
	 itO != mod->outcomes.end(); ++itO) {
      string state = lay->name + '.' + mod->actionName;
      for (map<string, const string*>::const_iterator it=act->parameters.begin();
	   it != act->parameters.end(); ++it)
	state += '$' + it->first + '=' + *(it->second);
      state += '.' + (*itO)->name;
      map<string, const string*> res(params);
      for (vector<string>::const_iterator itSVU = (*itO)->stateVariableUpdate.begin();
	   itSVU != (*itO)->stateVariableUpdate.end(); ++itSVU) {
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
	itS->second.prevOutcome = *itO;
	itS->second.stateVariables = res;
      } else {
	std::cout << "~state " << state << std::endl;
      }
      act->outcomes.insert(&(itS->second));
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
  if (stateVariableDomain == NULL) {
    stateVariableDomain = new map<string, domain_type>();
    pru.fillSVdomain(*stateVariableDomain);
  }
  this->stateVariableDomain = stateVariableDomain;
  states.insert( std::pair<string,MDPstate>("robotPos", MDPstate("robotPos")) );
  // TODO get actions for initial state
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
    } // for iMod in lay->modules
  } // for iLay in pru.layers
} // MDP(...)
