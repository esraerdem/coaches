#include "T42.h"

class PRUplanner : public T42 {
public:
  PRUplanner (ros::NodeHandle node);
  virtual void noMoveCallBack();
  virtual void plan();
};

PRUplanner::PRUplanner(ros::NodeHandle node) : T42(node) {
}

void PRUplanner::noMoveCallBack() {
  plan();
}

void PRUplanner::plan() {
  // Let's focus at first on going to site 3 (phone shop)
  const int TARGET = 3;
  int closestSite;
  if (! updateDistances(closestSite))
    return; // unable to update distances

  std::vector<std::vector<float> > rewards; // rewards[i][j] for going from i to j
  std::vector<float> init;
  std::vector<float> val;
  std::vector<int> act;
  for (int i=0; i<fixedSites.size(); i++) {
    std::vector<float> rew;
    for (int j=0; j<fixedSites.size(); j++) {
      if (i==j) 
	if (i==TARGET)
	  rew.push_back(+1000); // stay here
	else
	  rew.push_back(0);
      else if (i==TARGET) 
	  rew.push_back(-1000); // don't leave target
      else {
	if (j==TARGET) rew.push_back(1000 - site2siteDistance[i][j]);
	else rew.push_back(5 - site2siteDistance[i][j]);
      } // if (i!=j)
    } // for each fixedSites j
    if (closestSite<0) {
      // If robot not on a site
      if (i==TARGET) 
	rew.push_back(-1000 - robot2siteDistance[i]);
      else 
	rew.push_back(-robot2siteDistance[i]); // not going back to start!
      if (i==TARGET) 
	init.push_back(1000 - robot2siteDistance[i]);
      else 
	init.push_back(5 - robot2siteDistance[i]);
    }
    val.push_back(-1000);
    act.push_back(-1);
    rewards.push_back(rew);
  } // for each fixedSites i
  if (closestSite<0) {
    // If robot not on a site
    init.push_back(0);
    val.push_back(-1000);
    act.push_back(-1);
    rewards.push_back(init); // last state is current position
  }
  
  for (int h=0; h<val.size(); h++) {
    bool changed = false;
    for (int s=0; s<val.size(); s++) {
      float bestR = -1000000;
      float bestS = -1;
      for (int s2=0; s2<val.size(); s2++) {
	float r = rewards[s][s2] + val[s2];
	if (r > bestR) {
	  bestR = r;
	  bestS = s2;
	}
      } // for each state s2
      val[s] = bestR;
      act[s] = bestS;
    } // for each state s
  } // for each horizon h
  std::cout << "Optimal plan is \n";
  for (int s=0; s<fixedSites.size(); s++) {
    std::cout << "Location " << s << " (" << look4name(s) << ", " 
	      << sitesLocation[s].x << " x " << sitesLocation[s].y
	      << ") : Goto " << act[s] << " (" << val[s] << ")" << std::endl;
  }
  if (closestSite<0) {
    // If robot not on a site
    std::cout << "Robot at " << robot.x << " x " << robot.y
	      << " : Goto " << act.back() << " (" << val.back() << ")" << std::endl;
  } else {
    std::cout << "Robot is at location " << closestSite << " ("
	      << look4name(closestSite) << ")" << std::endl;
  }
} // plan()


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pruPlanner");

  ros::NodeHandle node;

  PRUplanner t42(node);
  t42.run();

  return 0;
}
