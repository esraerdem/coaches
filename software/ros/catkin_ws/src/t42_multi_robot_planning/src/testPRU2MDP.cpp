#include "PRU2MDP.h"
#include <locale.h>

int main() {
  std::locale::global(std::locale(""));
  setlocale(LC_NUMERIC,"C");

  // First reads the PRU
  PRUplus::actionsDomain["AvailableAds"].insert("Monoprix");
  PRUplus::actionsDomain["AvailableAds"].insert("Restaurant");
  PRUplus::actionsDomain["AvailableAds"].insert("DoorE");
  PRUplus pru("pru.xml");

  // Next builds a MDP from it

  PRU2MDP p2m(pru);
}
