#!/bin/bash

DIR=`pwd`

# make external modules
cd $COACHES_HOME/external/PetriNetPlans/PNP
if [ ! -d build ]; then
  mkdir build
  cd build
  cmake ..
  cd ..
fi
cd build
make install


# make ROS src 
cd $COACHES_HOME/ros/catkin_ws
catkin_make

cd $DIR


