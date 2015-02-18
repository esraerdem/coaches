#include <fstream>
#include <map>
#include <iostream>
#include <cstdlib>

#include "pnpgenerator.h"

int main(int argc, char **argv)
{

    std::map<string,string> policy;
    policy["RobotPos"] = "advertise_restaurant";
    policy["restaurant"] = "advertise_monoprix";
    policy["monoprix"] = "interact_doorEast";

    std::map<std::pair<string,string>,string> transition_fn;

    transition_fn[std::make_pair("doorWest","advertise_restaurant")] = "restaurant";
    transition_fn[std::make_pair("restaurant","advertise_monoprix")] = "monoprix";
    transition_fn[std::make_pair("monoprix","interact_doorEast")] = "doorEast";

    string initial_state = "RobotPos";
    string final_state = "doorEast";

    PNP pnp("policy");
    Place *p0 = pnp.addPlace("init"); p0->setInitialMarking();
    string current_state = initial_state;

    while (current_state!=final_state) {
        std::pair<Transition*,Place*> pa = pnp.addCondition("["+current_state+"]",p0);
        Place *p1 = pa.second;
        // std::cout << "curr: " << current_state << std::endl;
        string action = policy[current_state];
        if (action=="") {
            std::cerr << "ERROR. No action found at this point!!!" << std::endl;
            exit(-1);
        }
        // std::cout << action << std::endl;
        Place *p2 = pnp.addAction(action,p1);
        current_state = transition_fn[std::make_pair(current_state,action)];
        if (current_state=="") {
            std::cerr << "ERROR. No successor state found at this point!!!" << std::endl;
            exit(-1);
        }
        // std::cout << "next: " << current_state << std::endl << std::endl;
        p0 = p2;
    }
    std::pair<Transition*,Place*> pa = pnp.addCondition("["+current_state+"]",p0);
    pa.second->setName("goal");


/*
    PNP pnp("test");
    Place *p0 = pnp.addPlace("init"); p0->setInitialMarking();
    Place *p1 = pnp.addCondition("[location_A]",p0);
    Place *p2 = pnp.addAction("advertise_B",p1);
    Place *p3 = pnp.addCondition("[location_B]",p2);
    Place *p4 = pnp.addAction("advertise_C",p3);
    Place *p5 = pnp.addCondition("[location_C]",p4);
    Place *p6 = pnp.addAction("interact_G",p5);
    Place *p7 = pnp.addCondition("[location_G]",p6);
    p7->setName("goal");
*/

    std::ofstream of("out.pnml");
    of << pnp;
    of.close();

    return 0;
}

