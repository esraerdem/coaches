#!/bin/bash

# This file is called from the main coaches_init.bash script

# Initialize the workspace
if [ ! -d devel ]; then
    cd src
    catkin_init_workspace
    cd ..
fi

# Compile the workspace
catkin_make --pkg hello_coaches_developers

# Setting up environment variables
source $COACHES_HOME/ros/catkin_ws/devel/setup.bash

# Run hello
rosrun hello_coaches_developers hello


