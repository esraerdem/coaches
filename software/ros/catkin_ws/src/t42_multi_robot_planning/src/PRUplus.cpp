#include "PRUplus.h"

#include <iostream>
#include <stdlib.h>
#include <locale.h>
#include <boost/algorithm/string.hpp>
#include "MDP.h"

map<string, domain_type> actionsDomain;

static inline string trimString(xmlpp::TextReader &r) {
  string tmp = r.read_string();
  boost::algorithm::trim(tmp);
  return tmp;
}

static inline void readVector(vector<string> &v, xmlpp::TextReader &r, string sep="\n") {
  string tmp = r.read_string();
  vector<string> sv;
  boost::algorithm::split( sv, tmp, boost::algorithm::is_any_of(sep), boost::algorithm::token_compress_on );
  for (vector<string>::iterator it=sv.begin(); it != sv.end(); ++it) {
    tmp = *it;
    boost::algorithm::trim(tmp);
    if (! tmp.empty())
      v.push_back(tmp);
  }
}

static inline void readVector(set<string> &v, xmlpp::TextReader &r, string sep="\n") {
  string tmp = r.read_string();
  vector<string> sv;
  boost::algorithm::split( sv, tmp, boost::algorithm::is_any_of(sep), boost::algorithm::token_compress_on );
  for (vector<string>::iterator it=sv.begin(); it != sv.end(); ++it) {
    tmp = *it;
    boost::algorithm::trim(tmp);
    if (! tmp.empty())
      v.insert(tmp);
  }
}

PRUplus::PRUplus(string xmlFileName) {
  try {
    xmlpp::TextReader reader(xmlFileName.c_str());
    readXML(reader);
  } catch(const std::exception& e) {
    std::cerr << "Exception caught: " << e.what() << std::endl;
  }
}

void PRUplus::readXML(xmlpp::TextReader &reader) {
  while(reader.read()) {
    string name = reader.get_name();
    if ((reader.get_node_type() == xmlpp::TextReader::EndElement) &&
	(name == "pru"))
      break;
    if (reader.get_node_type() != xmlpp::TextReader::Element )
      continue;
    if (name == "pru") {
    } else if (name == "Start") {
    } else if (name == "SVU")
      readVector(stateVariablesInitialAssignments, reader);
    else if (name == "Next")
      readVector(firstEnabledModules, reader, "\n ");
    else if (name == "Layer")
      layers.push_back(new PRUlayer(reader));
    else
      std::cerr << "Unexpected tag " << name << "!" << std::endl;
  } // while reader.read()
} // PRUplus(reader)

PRUlayer::PRUlayer(xmlpp::TextReader &reader) {
  if (reader.has_attributes()) {
    reader.move_to_first_attribute();
    do {
      if (reader.get_name() == "id")
	name = reader.get_value();
    } while (reader.move_to_next_attribute());
    reader.move_to_element();
  }
  while(reader.read()) {
    string name = reader.get_name();
    if ((reader.get_node_type() == xmlpp::TextReader::EndElement) &&
	(name == "Layer"))
      break;
    if (reader.get_node_type() != xmlpp::TextReader::Element )
      continue;
    if (name == "StateVariable") {
      if (reader.has_attributes()) {
	reader.move_to_first_attribute();
	do {
	  if (reader.get_name() == "id")
	    stateVariables.push_back(reader.get_value());
	} while (reader.move_to_next_attribute());
	reader.move_to_element();
      }
    } else if (name == "Action") {
      modules.push_back(new PRUmodule(reader));
    } else
      std::cerr << "Unexpected tag " << name << "!" << std::endl;
  } // while reader.read()
} // PRUlayer(reader)

PRUmodule::PRUmodule(xmlpp::TextReader &reader) {
  if (reader.has_attributes()) {
    reader.move_to_first_attribute();
    do {
      if (reader.get_name() == "id")
	actionName = reader.get_value();
    } while (reader.move_to_next_attribute());
    reader.move_to_element();
  }
  while(reader.read()) {
    string name = reader.get_name();
    if ((reader.get_node_type() == xmlpp::TextReader::EndElement) &&
	(name == "Action"))
      break;
    if (reader.get_node_type() != xmlpp::TextReader::Element )
      continue;
    if (name == "Parameter") {
      string pName = "";
      string pDom = "";
      if (reader.has_attributes()) {
	reader.move_to_first_attribute();
	do {
	  if (reader.get_name() == "id")
	    pName = reader.get_value();
	  else if (reader.get_name() == "domain")
	    pDom = reader.get_value();
	} while (reader.move_to_next_attribute());
	reader.move_to_element();
      }
      if (pName == "")
	throw "Parameter with no name!";
      domain_type dom;
      if (pDom != "") {
	//TODO add all strings from this domain to this parameter
	if (actionsDomain[pDom].empty())
	  std::cerr << "Empty action domain: " << pDom << std::endl;
	else for (domain_type::const_iterator it=actionsDomain[pDom].begin(); 
	     it!=actionsDomain[pDom].end(); ++it){
	  dom.insert(*it);
	}
      }
      if (reader.has_value())
	dom.insert(reader.get_value());
      else
	readVector(dom,reader);
      parameters[pName] = dom;
    } else if (name == "Outcome") {
      outcomes.push_back(new PRUoutcome(reader));
    } else
      std::cerr << "Unexpected tag " << name << "!" << std::endl;
  } // while reader.read()
} // PRUmodule(reader)

PRUoutcome::PRUoutcome(xmlpp::TextReader &reader) {
  initDefaultValues();
  if (reader.has_attributes()) {
    reader.move_to_first_attribute();
    do {
      if (reader.get_name() == "id")
	name = reader.get_value();
      else if (reader.get_name() == "p")
	probability = atof(reader.get_value().c_str());
    } while (reader.move_to_next_attribute());
    reader.move_to_element();
  }
  while(reader.read()) {
    string name = reader.get_name();
    if ((reader.get_node_type() == xmlpp::TextReader::EndElement) &&
	(name == "Outcome"))
      break;
    if (reader.get_node_type() != xmlpp::TextReader::Element )
      continue;
    if (name == "Quality") {
      if (reader.has_attributes()) {
	reader.move_to_first_attribute();
	do {
	  if (reader.get_name() == "kind")
	    quality = reader.get_value();
	  else if (reader.get_name() == "param")
	    qualityParameter = atof(reader.get_value().c_str());
	  else if (reader.get_name() == "const")
	    qualityConstant = atof(reader.get_value().c_str());
	} while (reader.move_to_next_attribute());
	reader.move_to_element();
      }
    } else if (name == "Duration") {
      if (reader.has_attributes()) {
	reader.move_to_first_attribute();
	do {
	  if (reader.get_name() == "kind")
	    duration = reader.get_value();
	  else if (reader.get_name() == "param")
	    durationParameter = atof(reader.get_value().c_str());
	  else if (reader.get_name() == "const")
	    durationConstant = atof(reader.get_value().c_str());
	} while (reader.move_to_next_attribute());
	reader.move_to_element();
      }
    } else if (name == "Observe") {
      if (reader.has_value())
	observable = reader.get_value();
      else
	observable = reader.read_string();
    } else if (name == "SVU") {
      readVector(stateVariableUpdate,reader);
    } else if (name == "Final") {
      isFinal = true;
      if (reader.has_attributes()) {
	reader.move_to_first_attribute();
	do {
	  if (reader.get_name() == "label")
	    finalLabel = reader.get_value();
	} while (reader.move_to_next_attribute());
	reader.move_to_element();
      }
    } else if (name == "Next")
      readVector(nextModules,reader,"\n ");
    else
      std::cerr << "Unexpected tag " << name << "!" << std::endl;
  } // while reader.read()
} // PRUoutcome(reader)


void PRUplus::fillSVdomain (map<string, domain_type> &stateVariableDomain) const {
  for (vector<string>::const_iterator it = stateVariablesInitialAssignments.begin();
       it != stateVariablesInitialAssignments.end(); ++it) {
    vector<string> vec;
    boost::algorithm::split(vec, *it, boost::algorithm::is_any_of(":= "), 
			    boost::algorithm::token_compress_on );
    if (vec.size()!=2)
      std::cerr << "Unreadable SVU : " << *it << std::endl;
    else
      stateVariableDomain[vec[0]].insert(vec[1]);
  }
  for (vector<PRUlayer*>::const_iterator it = layers.begin(); it != layers.end(); ++it) {
    (*it)->fillSVdomain(stateVariableDomain);
  }
} // PRUplus::fillSVdomain(stateVariableDomain)

void PRUlayer::fillSVdomain (map<string, domain_type> &stateVariableDomain) const {
  for (vector<PRUmodule*>::const_iterator it = modules.begin(); 
       it != modules.end(); ++it) {
    (*it)->fillSVdomain(stateVariableDomain);
  }  
} // PRUlayer::fillSVdomain(stateVariableDomain)

void PRUmodule::fillSVdomain (map<string, domain_type> &stateVariableDomain) const {
  for (vector<PRUoutcome*>::const_iterator it = outcomes.begin(); 
       it != outcomes.end(); ++it) {
    (*it)->fillSVdomain(stateVariableDomain, parameters);
  }  
} // PRUmodule::fillSVdomain(stateVariableDomain)

void PRUoutcome::fillSVdomain (map<string, domain_type> &stateVariableDomain,
			       const map<string, domain_type> &parameters) const {
  for (vector<string>::const_iterator it = stateVariableUpdate.begin();
       it != stateVariableUpdate.end(); ++it) {
    vector<string> vec;
    boost::algorithm::split(vec, *it, boost::algorithm::is_any_of(":= "), 
			    boost::algorithm::token_compress_on );
    if (vec.size()!=2)
      std::cerr << "Unreadable SVU : " << *it << std::endl;
    else {
      if (vec[1][0] == '$') {
	map<string, domain_type>::const_iterator itP = parameters.find(vec[1].substr(1));
	if (itP == parameters.end())
	  std::cerr << "Unknown module parameter: " << vec[1] << std::endl;
	else {
	  domain_type dom = itP->second;
	  for (domain_type::const_iterator itD = dom.begin(); 
	       itD != dom.end(); ++itD) {
	    stateVariableDomain[vec[0]].insert(*itD);
	  } // for itD in dom
	} // if known action parameter
      } // if assignment from action parameter
      else 
	stateVariableDomain[vec[0]].insert(vec[1]);
    } // if correct assignment var := value
  } // for it in stateVariableUpdate
} // PRUoutcome::fillSVdomain(stateVariableDomain,parameters)

float distanceFunction(const PRUstate& fromState,
		       const PRUmodule& withModule,
		       const PRUstate& toState,
		       const string& kind,
		       float parameter) {
  if (kind == "null")
    return 0;
  if (kind == "distance")
    return 0; // TODO compute the distance
  return -1;
}

void testPRUplus() {
  actionsDomain["AvailableAds"].insert("Monoprix");
  actionsDomain["AvailableAds"].insert("Restaurant");
  actionsDomain["AvailableAds"].insert("DoorE");
  PRUplus pru("pru.xml");
  std::cout << pru << std::endl;

  map<string, domain_type> stateVariableDomain;
  pru.fillSVdomain(stateVariableDomain);  
  std::cout << stateVariableDomain << std::endl;

  MDP m(pru, &stateVariableDomain);
  
}

int main() {
  std::locale::global(std::locale(""));
  setlocale(LC_NUMERIC,"C");

  testPRUplus();
}
