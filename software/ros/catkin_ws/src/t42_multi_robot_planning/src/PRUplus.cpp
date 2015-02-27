#include "PRUplus.h"
#include <libxml++/libxml++.h>
#include <libxml++/parsers/textreader.h>

#include <iostream>
#include <stdlib.h>

map<string, domain_type> allDomains;

PRUplus* readXML(string fileName) {
  std::locale::global(std::locale(""));
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
	res->stateVariablesInitialAssignments.push_back(reader.read_string());
      else if (name == "Next")
	res->firstEnabledModules.push_back(reader.read_string());
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
	  dom.push_back(reader.read_string());
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
	o->stateVariableUpdate.push_back(reader.read_string());
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
	o->nextModules.push_back(reader.read_string());
    } // while reader.read()
  } catch(const std::exception& e) {
    std::cerr << "Exception caught: " << e.what() << std::endl;
    return NULL;
  }
  return res;
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
}
