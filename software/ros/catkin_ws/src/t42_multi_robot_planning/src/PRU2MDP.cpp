
#include "PRU2MDP.h"
#include "MDP.h"

void PRU2MDPprogress::buildActions(const PRUmodule *mod, 
				   map<string,domain_type>::const_iterator iParam,
				   const MDPaction &act) {
  if (iParam == mod->parameters.end()) {
    // All parameters are instanciated
    actions.push_back(new MDPaction(act));
  } else {
    // Some action parameter is to be instanciated yet
    string name = iParam->first;
    domain_type::const_iterator it = iParam->second.begin();
    domain_type::const_iterator itEnd = iParam->second.end();
    ++iParam;
    for (;it != itEnd; ++it) {
      MDPaction ma(act,name,&*it);
      buildActions(mod, iParam, ma);
    } // for *it in the current parameter's domain
    --iParam;
  } // if more parameters
} // buildActions(*mod, iParam)

PRU2MDPprogress::PRU2MDPprogress(const PRUlayer *l, const map<string, 
				 const string*> &sv) : stateVariables(sv) {
  lay = l;
  for (vector<PRUmodule*>::const_iterator iMod = lay->modules.begin(); 
       iMod != lay->modules.end(); ++iMod) {
    PRUmodule *mod = *iMod;
    MDPaction act(mod->actionName); // template for building all those actions
    buildActions(mod, mod->parameters.begin(), act);
  } // for *iMod in lay->modules
} // PRU2MDPprogress(*l,&sv)

std::ostream& operator<<(std::ostream& os, const PRU2MDPprogress& progress) {
  os << "Layer " << progress.lay->name << " [";
  for (map<string, const string*>::const_iterator itSV = progress.stateVariables.begin();
       itSV != progress.stateVariables.end(); ++ itSV) {
    os << ' ' << itSV->first << '=' << *(itSV->second);
  } // for *itSV in stateVariables
  os << " ]" << std::endl;
  for (vector<MDPaction*>::const_iterator itA = progress.actions.begin();
       itA != progress.actions.end(); ++itA) {
    MDPaction *pa = *itA;
    MDPaction a = *pa;
    os << " > " << a << std::endl;
  }
  return os;
}

void PRU2MDP::buildProgress(const PRUlayer *layer,
			    vector<string>::const_iterator itSV, 
			    map<string, const string*> &params) {
  if (itSV == layer->stateVariables.end()) {
    // All variables are instanciated
    progress.push_back(new PRU2MDPprogress(layer,params));
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
  stateVariableDomain = new map<string, domain_type>();
  pru.fillSVdomain(*stateVariableDomain);
  for (vector<PRUlayer*>::const_iterator iLay = pru.layers.begin(); 
       iLay != pru.layers.end(); ++iLay) {
    PRUlayer *lay = *iLay;  
    map<string,const string*> params;
    buildProgress(lay, lay->stateVariables.begin(), params);
  } // for *iLay in pru.layers
} // PRU2MDP(&pru)
