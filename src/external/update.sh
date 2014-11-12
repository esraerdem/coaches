#!/bin/bash

# gradient_based_navigation
if [ ! -d gradient_based_navigation ]; then
    git clone https://github.com/Imperoli/gradient_based_navigation
else
    cd gradient_based_navigation; git pull; cd ..
fi

# PetriNetPlans

if [ ! -d PetriNetPlans ]; then
    git clone https://github.com/iocchi/PetriNetPlans
else
    cd PetriNetPlans; git pull; cd ..
fi


