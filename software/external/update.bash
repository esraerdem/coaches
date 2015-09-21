#!/bin/bash

# This file is called from the main coaches_update.bash script

# gradient_based_navigation
if [ ! -d gradient_based_navigation ]; then
    git clone https://github.com/Imperoli/gradient_based_navigation
    cd ../ros/catkin_ws/src/
    ln -sf ../../../external/gradient_based_navigation .
    cd -
else
    cd gradient_based_navigation; git pull; cd ..
fi

# PetriNetPlans

if [ ! -d PetriNetPlans ]; then
    git clone https://github.com/iocchi/PetriNetPlans
    cd ../ros/catkin_ws/src/
    ln -sf ../../../external/PetriNetPlans/PNPros/ROS_bridge/* .
    cd -
else
    cd PetriNetPlans; git pull; cd ..
fi


# Stage environments

if [ ! -d stage_environments ]; then
    svn co https://labrococo.dis.uniroma1.it/svn/software-open/trunk/rococo-ros/stage_environments/
    cd ../ros/catkin_ws/src/
    ln -sf ../../../external/stage_environments .
    cd -
else
    cd stage_environments; svn up; cd ..
fi

# glocalizer

if [ ! -d glocalizer ]; then
    svn co https://labrococo.dis.uniroma1.it/svn/software-open/trunk/rococo-ros/glocalizer/
    cd ../ros/catkin_ws/src/
    ln -sf ../../../external/glocalizer .
    cd -
else
    cd rococo_navigation; svn up; cd ..
fi

# Rococo Navigation

if [ ! -d rococo_navigation ]; then
    svn co https://labrococo.dis.uniroma1.it/svn/software-open/trunk/rococo-ros/rococo_navigation/
    cd ../ros/catkin_ws/src/
    ln -sf ../../../external/rococo_navigation .
    cd -
else
    cd rococo_navigation; svn up; cd ..
fi

# Rococo Laser Analysis

if [ ! -d laser_analysis ]; then
    svn co https://labrococo.dis.uniroma1.it/svn/software-open/trunk/rococo-ros/laser_analysis/
    cd ../ros/catkin_ws/src/
    ln -sf ../../../external/laser_analysis .
    cd -
else
    cd laser_analysis; svn up; cd ..
fi

# TCP interface

if [ ! -d tcp_interface ]; then
    git clone https://github.com/gennari/tcp_interface
    cd ../ros/catkin_ws/src/
    ln -sf ../../../external/tcp_interface .
    cd -
else
    cd tcp_interface; git pull; cd ..
fi

# thin navigation

# ...


