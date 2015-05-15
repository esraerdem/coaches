#!/bin/bash

# install new packages
if ! hash pico2wave 2>/dev/null; then
  echo "sudo apt-get install libttspico-utils"
  sudo apt-get install libttspico-utils
fi

if [[ -z $(find /usr/include/libxml++-2.6) ]]; then
  echo "sudo apt-get install libxml++2-6-dev"
  sudo apt-get install libxml++2.6-dev
fi

# update external modules
cd $COACHES_HOME/external
./update.bash
cd -

# update ROS src 
cd $COACHES_HOME/ros/catkin_ws/src
git pull
cd -


