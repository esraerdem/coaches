#!/bin/bash

if [ -z "$COACHES_HOME" ]; then
    echo "Please set COACHES_HOME environment variable in your .bashrc file to the coaches/src directory!"
    exit 1
fi

PATH=$PATH:$COACHES_HOME/bin

if [ -d $COACHES_HOME/ros/catkin_ws/devel ]; then
    source $COACHES_HOME/ros/catkin_ws/devel/setup.bash
fi

export PNP_INCLUDE=$COACHES_HOME/external/PetriNetPlans/PNP/include
export PNP_LIB=$COACHES_HOME/external/PetriNetPlans/PNP/lib


echo "COACHES setup completed."