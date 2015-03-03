#include "PRUplus.h"
#include <libxml++/libxml++.h>
#include <libxml++/parsers/textreader.h>

#include <iostream>
#include <stdlib.h>
#include <locale.h>
#include <boost/algorithm/string.hpp>

map<string, domain_type> allDomains;

static inline string trimString(xmlpp::TextReader &r) {
  string tmp = r.read_string();
  boost::algorithm::trim(tmp);
  return tmp;
}

PRUplus* readXML(string fileName) {
  std::locale::global(std::locale(""));
  setlocale(LC_NUMERIC,"C");

  PRUplus *res = new PRUplus();

  try {
    xmlpp::TextReader reader(fileName.c_str());
    // Reading the first part
    while(reader.read()) {
      //      std::cout << reader.get_name() << ":" << reader.get_node_type() << std::endl;
      string name = reader.get_name();
      if ((reader.get_node_type() == xmlpp::TextReader::EndElement) &&
	  (name == "Start"))
	break;
      if (reader.get_node_type() != xmlpp::TextReader::Element )
	continue;
      if (name == "pru")
	res = new PRUplus();
      else if (name == "Start")
	{}
      else if (name == "SVU")
	res->stateVariablesInitialAssignments.push_back(trimString(reader));
      else if (name == "Next")
	res->firstEnabledModules.push_back(trimString(reader));
    } // while reader.read()

    // reading the main part
    PRUlayer *l;
    PRUmodule *a;
    PRUoutcome *o;
    while(reader.read()) {
      //      std::cout << reader.get_name() << ":" << reader.get_node_type() << std::endl;
      string name = reader.get_name();
      if ((reader.get_node_type() == xmlpp::TextReader::EndElement) &&
	  (name == "pru"))
	break;
      if (reader.get_node_type() != xmlpp::TextReader::Element )
	continue;
      if (name == "Layer") {
	l = new PRUlayer;
	res->layers.push_back(l);
	if (reader.has_attributes()) {
	  reader.move_to_first_attribute();
	  do {
	    if (reader.get_name() == "id")
	      l->name = reader.get_value();
	  } while (reader.move_to_next_attribute());
	  reader.move_to_element();
	}
      } else if (name == "StateVariable") {
	if (reader.has_attributes()) {
	  reader.move_to_first_attribute();
	  do {
	    if (reader.get_name() == "id")
	      l->stateVariables.push_back(reader.get_value());
	  } while (reader.move_to_next_attribute());
	  reader.move_to_element();
	}
      } else if (name == "Action") {
	a = new PRUmodule();
	if (reader.has_attributes()) {
	  reader.move_to_first_attribute();
	  do {
	    if (reader.get_name() == "id")
	      a->actionName = reader.get_value();
	  } while (reader.move_to_next_attribute());
	  reader.move_to_element();
	}
	l->modules.push_back(a);
      } else if (name == "Parameter") {
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
	}
	if (reader.has_value())
	  dom.push_back(reader.get_value());
	else
	  dom.push_back(trimString(reader));
	a->parameters[pName] = dom;
      } else if (name == "Outcome") {
	o = new PRUoutcome();
	if (reader.has_attributes()) {
	  reader.move_to_first_attribute();
	  do {
	    if (reader.get_name() == "id")
	      o->name = reader.get_value();
	    else if (reader.get_name() == "p")
	      o->probability = atof(reader.get_value().c_str());
	  } while (reader.move_to_next_attribute());
	  reader.move_to_element();
	}
	a->outcomes.push_back(o);
      } else if (name == "Quality") {
	if (reader.has_attributes()) {
	  reader.move_to_first_attribute();
	  do {
	    if (reader.get_name() == "kind")
	      o->quality = reader.get_value();
	    else if (reader.get_name() == "param")
	      o->qualityParameter = atof(reader.get_value().c_str());
	    else if (reader.get_name() == "const")
	      o->qualityConstant = atof(reader.get_value().c_str());
	  } while (reader.move_to_next_attribute());
	  reader.move_to_element();
	}
      } else if (name == "Duration") {
	if (reader.has_attributes()) {
	  reader.move_to_first_attribute();
	  do {
	    if (reader.get_name() == "kind")
	      o->duration = reader.get_value();
	    else if (reader.get_name() == "param")
	      o->durationParameter = atof(reader.get_value().c_str());
	    else if (reader.get_name() == "const")
	      o->durationConstant = atof(reader.get_value().c_str());
	  } while (reader.move_to_next_attribute());
	  reader.move_to_element();
	}
      } else if (name == "Observe") {
	if (reader.has_value())
	  o->observable = reader.get_value();
	else
	  o->observable = reader.read_string();
      } else if (name == "SVU") {
	o->stateVariableUpdate.push_back(trimString(reader));
      } else if (name == "Final") {
	o->isFinal = true;
	if (reader.has_attributes()) {
	  reader.move_to_first_attribute();
	  do {
	    if (reader.get_name() == "label")
	      o->finalLabel = reader.get_value();
	  } while (reader.move_to_next_attribute());
	  reader.move_to_element();
	}	
      } else if (name == "Next")
	o->nextModules.push_back(trimString(reader));
    } // while reader.read()
  } catch(const std::exception& e) {
    std::cerr << "Exception caught: " << e.what() << std::endl;
    return NULL;
  }
  return res;
} // readXML()

std::ostream& operator<<(std::ostream& os, const PRUoutcome& option) {
  os << option.name << " (" << (option.probability*100) <<"%): "
     << option.observable 
     << "\n      Q=" << option.quality << "(" << option.qualityParameter << ")+"
     << option.qualityConstant
     << "\n      D=" << option.duration << "(" << option.durationParameter << ")+"
     << option.durationConstant;
  os << "\n      SVU{";
  for (vector<string>::const_iterator it = option.stateVariableUpdate.begin();
       it != option.stateVariableUpdate.end(); ++it) {
    os << "\n       " << *it;
  } // for *it in option.SVU
  os << "\n      }";
  os << "\n      NEXT{";
  for (vector<string>::const_iterator it = option.nextModules.begin();
       it != option.nextModules.end(); ++it) {
    os << "\n       " << *it;
  } // for *it in pru.firstEnabledModules
  os << "\n      }";
  if (option.isFinal)
    os << "\n      FINAL Label=" << option.finalLabel;
  return os;
}

std::ostream& operator<<(std::ostream& os, const PRUmodule& module) {
  os << "  MODULE " << module.actionName << " {\n";
  os << "   PARAM{\n";
  for (map<string, domain_type>::const_iterator it = module.parameters.begin();
       it != module.parameters.end(); ++it) {
    os << "    " << it->first << " in {" ;
    for (vector<string>::const_iterator it2 = it->second.begin();
	 it2 != it->second.end(); ++it2) {
      os << " " << *it2 ;
    } // for *it2 in parameter *it 's domain
    os << "   }\n";
  } // for *it in module.parameters
  os << "   }\n";
  for (vector<PRUoutcome*>::const_iterator it = module.outcomes.begin();
       it != module.outcomes.end(); ++it) {
    os << "   > " << **it << "\n";
  } // for *it in module.outcomes
  os << "  }\n";
  return os;
}

std::ostream& operator<<(std::ostream& os, const PRUlayer& layer) {
  os << " LAYER " << layer.name << " {\n";
  os << "  VARS{";
  for (vector<string>::const_iterator it = layer.stateVariables.begin();
       it != layer.stateVariables.end(); ++it) {
    os << " " << *it ;
  } // for *it in layer.stateVariables
  os << " }\n";
  for (vector<PRUmodule*>::const_iterator it = layer.modules.begin();
       it != layer.modules.end(); ++it) {
    os << **it;
  } // for *it in layer.modules
  os << " }\n";
  return os;
}

std::ostream& operator<<(std::ostream& os, const PRUplus& pru) {
  os << "PRU{\n INIT{\n";
  for (vector<string>::const_iterator it = pru.stateVariablesInitialAssignments.begin();
       it != pru.stateVariablesInitialAssignments.end(); ++it) {
    os << "  " << *it << "\n";
  } // for *it in pru.stateVariablesInitialAssignments
  os << " }\n NEXT{\n";
  for (vector<string>::const_iterator it = pru.firstEnabledModules.begin();
       it != pru.firstEnabledModules.end(); ++it) {
    os << "  " << *it << "\n";
  } // for *it in pru.firstEnabledModules
  os << " }\n";
  for (vector<PRUlayer*>::const_iterator it = pru.layers.begin();
       it != pru.layers.end(); ++it) {
    os << **it;
  } // for *it in pru.layers
  os << "}\n";
  return os;
}

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

int main() {
  PRUplus *pru = readXML("pru.xml");
  std::cout << *pru << std::endl;
}
