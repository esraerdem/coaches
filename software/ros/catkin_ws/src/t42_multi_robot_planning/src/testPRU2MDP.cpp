#include "PRU2MDP.h"
#include <locale.h>

static float testFunction(const PRUstate& fromState, const PRUstate& toState,
		   const string& kind, float parameter) {
  if (kind == "null")
    return 0;
  if (kind == "distance")
    return 0; 
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
}
