#!/bin/bash

# This file is called from the main update.bash script in coaches/src

# gradient_based_navigation
if [ ! -d gradient_based_navigation ]; then
    git clone https://github.com/Imperoli/gradient_based_navigation
    cd ../ros/catkin_ws/src/
    ln -sf ../../../external/gradient_based_navigation .
    cd -
# ../../../external/
else
    cd gradient_based_navigation; git pull; cd ..
fi

# PetriNetPlans

if [ ! -d PetriNetPlans ]; then
    git clone https://github.com/iocchi/PetriNetPlans
    cd ../ros/catkin_ws/src/
    ln -sf ../../../external/PetriNetPlans/PNPros .
    cd - 
#../../../external/
else
    cd PetriNetPlans; git pull; cd ..
fi


