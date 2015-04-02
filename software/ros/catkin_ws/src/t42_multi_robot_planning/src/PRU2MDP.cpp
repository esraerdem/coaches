
#include "PRU2MDP.h"
#include "MDP.h"
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

void PRU2MDPprogress::registerActions(map<string, PRU2MDPactionDescriptor> &allActions) const {
  string prefix = lay->name + ".";
  string suffix = ".";
  for (map<string, const string*>::const_iterator it = stateVariables.begin();
       it != stateVariables.end(); ++it) 
      suffix += '$' + it->first + '=' + *(it->second);
  for (vector<MDPaction*>::const_iterator itA = actions.begin();
       itA != actions.end(); ++itA) {
    MDPaction *act = *itA;
    string infix = act->actionName;
    for (map<string, const string*>::const_iterator it=act->parameters.begin();
       it != act->parameters.end(); ++it)
      infix += ',' + it->first + '=' + *(it->second);
    allActions.insert(std::pair<string,PRU2MDPactionDescriptor>
		      (prefix+infix+suffix, PRU2MDPactionDescriptor(this, act)));
  } // for *itA in actions
} // registerActions(&allActions)

void PRU2MDPprogress::buildActions(const PRUmodule *mod, 
				   map<string,domain_type>::const_iterator iParam,
				   const MDPaction &action, 
				   map<string, domain_type> *SVdomain) {
  if (iParam == mod->parameters.end()) {
    // All parameters are instanciated
    MDPaction *act = new MDPaction(action);
    actions.push_back(act);
    
    map<string, const string*> res(stateVariables); // makes a local copy of SV that will be updated with out SVUs
    for (vector<PRUoutcome*>::const_iterator itO = mod->outcomes.begin();
	 itO != mod->outcomes.end(); ++itO) {
      PRUoutcome *out = *itO;
      for (vector<string>::const_iterator itSVU = out->stateVariableUpdate.begin();
	   itSVU != out->stateVariableUpdate.end(); ++itSVU) {
	vector<string> vec;
	boost::algorithm::split(vec, *itSVU, boost::algorithm::is_any_of(":= "), 
				boost::algorithm::token_compress_on );
	if (vec.size()!=2)
	  std::cerr << "Unreadable SVU : " << *itSVU << std::endl;
	else {
	  if (vec[1][0] == '$') {
	    // this is an action parameter
	    string p = vec[1].substr(1);
	    const string *v = act->getParameter(p);
	    if (v == NULL)
	      std::cerr << "Unknown action parameter "<<vec[1]<<std::endl;
	    else
	      res[vec[0]] = v;
	  } else {
	    // res[vec[0]] = vec[1]; // but needs a pointer (no local string !)
	    domain_type &dom = (*SVdomain)[vec[0]];
	    for (domain_type::const_iterator it = dom.begin();
		 it != dom.end(); ++it) {
	      if (*it == vec[1]) {
		res[vec[0]] = &(*it);
		break;
	      }
	    } // for *it in this SV domain
	  } // if not an action parameter
	} // if correct SVU element
      } // for itSVU in *itO SV Updates
      // Here, res contains the updated state variables
      MDPstate *s = new MDPstate(lay->name, act, out, res);
      act->outcomes.insert(s);
    } // for *itO in mod->outcomes
  } else {
    // Some action parameter is to be instanciated yet
    string name = iParam->first;
    domain_type::const_iterator it = iParam->second.begin();
    domain_type::const_iterator itEnd = iParam->second.end();
    ++iParam;
    for (;it != itEnd; ++it) {
      MDPaction ma(action,name,&*it);
      buildActions(mod, iParam, ma, SVdomain);
    } // for *it in the current parameter's domain
    --iParam;
  } // if more parameters
} // buildActions(*mod, iParam)

PRU2MDPprogress::PRU2MDPprogress(const PRUlayer *l, const map<string, 
				 const string*> &sv,
				 map<string, domain_type> *stateVariableDomain) 
  : stateVariables(sv) {
  lay = l;
  for (vector<PRUmodule*>::const_iterator iMod = lay->modules.begin(); 
       iMod != lay->modules.end(); ++iMod) {
    PRUmodule *mod = *iMod;
    MDPaction act(mod->actionName); // template for building all those actions
    buildActions(mod, mod->parameters.begin(), act, stateVariableDomain);
  } // for *iMod in lay->modules
} // PRU2MDPprogress(*l,&sv)

PRU2MDPprogress::~PRU2MDPprogress(){
  for (vector<MDPaction*>::iterator itA = actions.begin();
       itA != actions.end(); ++itA) {
    std::cout << "Temporary destruction of " << **itA << std::endl;
    delete (*itA);
  }
}

bool PRU2MDPprogress::isMatching(const map<string, const string*> &stateVariables) const {
  for (map<string, const string*>::const_iterator itV = this->stateVariables.begin();
       itV != this->stateVariables.end(); ++itV) {
    map<string, const string*>::const_iterator itF = stateVariables.find(itV->first);
    if (itF == stateVariables.end())
      return false;
    if (*itF->second != *itV->second)
      return false;
  }
  return true; // layer name matches and any specified variable does...
} // isMatching(...)

std::ostream& operator<<(std::ostream& os, const PRU2MDPprogress& progress) {
  os << "Layer " << progress.lay->name << " [";
  for (map<string, const string*>::const_iterator itSV = progress.stateVariables.begin();
       itSV != progress.stateVariables.end(); ++ itSV) {
    os << ' ' << itSV->first << '=' << *(itSV->second);
  } // for *itSV in stateVariables
  os << " ]" << std::endl;
  for (vector<MDPaction*>::const_iterator itA = progress.actions.begin();
       itA != progress.actions.end(); ++itA) {
    os << " > " << **itA << std::endl;
  }
  return os;
}


void PRU2MDP::buildProgress(const PRUlayer *layer,
			    vector<string>::const_iterator itSV, 
			    map<string, const string*> &params){
  if (itSV == layer->stateVariables.end()) {
    // All variables are instanciated
    progress.push_back(new PRU2MDPprogress(layer,params, stateVariableDomain));
    progress.back()->registerActions(allActions);
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
	buildProgress(layer,itSV,params);
      } // for itD in variable domain
      --itSV;
      params.erase(var);
    } // if known state variable
  } // if some state variable left
} // buildProgress(*layer)

PRU2MDP::PRU2MDP(const PRUplus &pru) {
  // First compute State Variables Domain
  stateVariableDomain = new map<string, domain_type>();
  pru.fillSVdomain(*stateVariableDomain);

  // Next builds progress (along with actions and states)
  for (vector<PRUlayer*>::const_iterator iLay = pru.layers.begin(); 
       iLay != pru.layers.end(); ++iLay) {
    PRUlayer *lay = *iLay;  
    map<string,const string*> params;
    buildProgress(lay, lay->stateVariables.begin(), params);
  } // for *iLay in pru.layers

  for (map<string,PRU2MDPactionDescriptor>::const_iterator it=allActions.begin();
       it != allActions.end(); ++it) {
    std::cout << it->first << std::endl;
  }
  // Finally matches next-module with MDPaction in each state
  for (vector<PRU2MDPprogress*>::iterator itP=progress.begin();
       itP != progress.end(); ++itP) {
    (*itP)->matchActions(this);
  } // for *itP in progress
} // PRU2MDP(&pru)

void PRU2MDPprogress::matchActions(const PRU2MDP *model) {
  for(vector<MDPaction*>::iterator itA = actions.begin();
      itA != actions.end(); ++itA) {
    MDPaction *act = *itA;
    for(set<MDPstate*>::iterator itO = act->outcomes.begin();
	itO != act->outcomes.end(); ++itO) {
      MDPstate *s = *itO;
      matchActions(s->prevOutcome->nextModules, s, model);
    } // for s=*itO in act->outcomes
  } // for act=*itA in actions
} // matchActions(*model)

void PRU2MDPprogress::matchActions(const vector<string> &nextModules, MDPstate *s, const PRU2MDP *model) {
  for(vector<string>::const_iterator itM = nextModules.begin();
      itM != nextModules.end(); ++itM) {
    string module = *itM;
    boost::regex expression("(^|\b)"+module+"(,|$)"); // adds the word-boundary conditions
    if (module[0]=='-')
      expression = boost::regex("(^|\b)"+module.substr(1)+"(,|$)");
    std::cout << "Matching " << module << std::endl;
    for (map<string, PRU2MDPactionDescriptor>::const_iterator it = model->allActions.begin();
	 it != model->allActions.end(); ++it) {
      if (it->second.progress->isMatching(s->stateVariables))
	if (regex_search(it->first, expression)) {
	  if (module[0]=='-') {
	    std::cout << " < " << it->first << std::endl;
	    s->availableActions.erase(it->second.action);
	  } else {
	    std::cout << " > " << it->first << std::endl;
	    s->availableActions.insert(it->second.action);
	  }
	} // if match
    } // for *it in model->allActions
  } // for module=*itM in state.prevOutcome->nextModules
} // matchActions(&nextModules, *s, *model)

