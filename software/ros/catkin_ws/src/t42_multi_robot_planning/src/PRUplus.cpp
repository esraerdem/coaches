#include "PRUplus.h"

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

static inline void readVector(vector<string> &v, xmlpp::TextReader &r) {
  string tmp = r.read_string();
  vector<string> sv;
  boost::algorithm::split( sv, tmp, boost::algorithm::is_any_of("\n"), boost::algorithm::token_compress_on );
  for (vector<string>::iterator it=sv.begin(); it != sv.end(); ++it) {
    tmp = *it;
    boost::algorithm::trim(tmp);
    if (! tmp.empty())
      v.push_back(tmp);
  }
}

PRUplus::PRUplus(string xmlFileName) {
  try {
    xmlpp::TextReader reader(xmlFileName.c_str());
    readXML(reader);
  } catch(const std::exception& e) {
    std::cerr << "Exception caught: " << e.what() << std::endl;
  }
  std::cout << *this << std::endl;
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
      readVector(firstEnabledModules, reader);
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
      }
      if (reader.has_value())
	dom.push_back(reader.get_value());
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
      readVector(nextModules,reader);
    else
      std::cerr << "Unexpected tag " << name << "!" << std::endl;
  } // while reader.read()
} // PRUoutcome(reader)

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
	readVector(res->stateVariablesInitialAssignments,reader);
      else if (name == "Next")
	readVector(res->firstEnabledModules,reader);
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
	  readVector(dom,reader);
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
	readVector(o->stateVariableUpdate,reader);
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
	readVector(o->nextModules,reader);
    } // while reader.read()
  } catch(const std::exception& e) {
    std::cerr << "Exception caught: " << e.what() << std::endl;
    return NULL;
  }
  return res;
} // readXML()

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

void test() {
  PRUplus pru2 = PRUplus("pru.xml");
  std::cout << pru2 << std::endl;
}

int main() {
  std::locale::global(std::locale(""));
  setlocale(LC_NUMERIC,"C");
  /*
  PRUplus *pru = readXML("pru.xml");
  std::cout << *pru << std::endl;
  */

  test();
}
