#include "PRU2MDP.h"
#include "MDP.h"
#include <locale.h>

static float testFunction(const PRUstate& fromState, const PRUstate& toState,
		   const string& kind, float parameter) {
  if (kind == "null")
    return 0;
  if (kind == "distance") {
    PRUstate::const_iterator itF = fromState.find("location");
    PRUstate::const_iterator itT = toState.find("location");
    if ((itF==fromState.end()) || (itT==toState.end()))
      std::cerr<<"Unable to compute distance"<<std::endl;
    else {
      string f=*itF->second;
      string l=*itT->second;
      if (f=="RobotPos") {
	f = "Restaurant";
      }
      if (f == l)
	return 0;
      if (f < l) {
	f = *itF->second; l = *itT->second;
      } else {
	f = *itT->second; l = *itF->second;
      }

      if (f == "Carpark")
	if (l == "DoorE")
	  return 68;
	else if (l == "DoorW")
	  return 22;
	else if (l == "Monoprix")
	  return 51;
	else if (l == "Phone")
	  return 63;
	else
	  return 18;
      else if (f == "DoorE")
	if (l == "DoorW")
	  return 89;
	else if (l == "Monoprix")
	  return 17;
	else if (l == "Phone")
	  return 4;
	else
	  return 84;
      else if (f == "DoorW")
	if (l == "Monoprix")
	  return 72;
	else if (l == "Phone")
	  return 84;
	else
	  return 5;
      else if (f == "Monoprix")
	if (l == "Phone")
	  return 13;
	else
	  return 68;
      else if (f == "Phone")
	return 79;
    } // if both locations are defined
    std::cerr << "Unknown path from " << *itF->second << " to " << *itT->second << std::endl;
    return 0; 
  }
  return -1;
}

int main() {
  std::locale::global(std::locale(""));
  setlocale(LC_NUMERIC,"C");

  PRUplus::domainFunction = &testFunction;
  PRUplus::actionsDomain["AvailableAds"].insert("Monoprix");
  PRUplus::actionsDomain["AvailableAds"].insert("Restaurant");
  PRUplus::actionsDomain["AvailableAds"].insert("DoorE");
  // First reads the PRU
  PRUplus pru("pru.xml");

  // Next builds a MDP from it
  PRU2MDP p2m(pru);

  p2m.states.print();
  
  MDP *mdp = p2m.getMDP();
  mdp->initVI();
  std::cout << "Nb states : " << mdp->getStates().size() << std::endl;
  std::cout << "Init  state : " << mdp->getState(0)->getPredicates() << std::endl;
  std::cout << "First state : " << mdp->getState(1)->getPredicates() << std::endl;
  std::cout << "Fifth state : " << mdp->getState(5)->getPredicates() << std::endl;
  
  std::cout << "Solve : " << mdp->iterate(0.99f) << std::endl;
  mdp->printPolicy(std::cout);
  std::cout << "Solve : " << mdp->iterate(0.99f) << std::endl;
  mdp->printPolicy(std::cout);
  std::cout << "Solve : " << mdp->iterate(0.99f) << std::endl;
  mdp->printPolicy(std::cout);
  std::cout << "Solve : " << mdp->iterate(0.99f) << std::endl;
  mdp->printPolicy(std::cout);
  std::cout << "Solve : " << mdp->iterate(0.99f) << std::endl;
  mdp->printPolicy(std::cout);

  delete mdp;
}
