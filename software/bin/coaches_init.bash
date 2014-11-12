#!/bin/bash

if [ -z "$COACHES_HOME" ]; then
    echo "Please set COACHES_HOME environment variable in your .bashrc file to the coaches/src directory!"
    exit 1
fi

# Make dirs if missing
mkdir -p lib
mkdir -p include

# Install required Linux libs and applications
sudo apt-get install cmake g++ xterm libxml2 libxml2-dev flex

# Init Catkin workspace
cd ros/catkin_ws
./initialize.bash
cd -

