#!/bin/bash

# install new packages
if ! hash pico2wave 2>/dev/null; then
  sudo apt-get install libttspico-utils
fi

# update external modules
cd $COACHES_HOME/external
./update.bash
cd -

# update ROS src 
cd $COACHES_HOME/ros/catkin_ws/src
git pull
cd -


