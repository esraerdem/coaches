#!/bin/bash

# update external modules
cd $COACHES_HOME/external
./update.bash
cd -

# update ROS src 
cd $COACHES_HOME/ros/catkin_ws/src
git pull
cd -


