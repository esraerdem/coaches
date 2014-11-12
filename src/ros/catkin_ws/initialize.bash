#!/bin/bash

# This file is called from the main initialize.bash script in coaches/src

# Initialize the workspace
if [ ! -d devel ]; then
    cd src
    catkin_init_workspace
    cd ..
fi

# Compile the workspace
catkin_make

# Setting up environment variables
source devel/setup.bash

# Run hello
rosrun hello_coaches_developers hello


