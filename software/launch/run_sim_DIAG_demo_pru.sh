#!/bin/bash

DEMO_PRU=pruDIAG.xml
#DEMO_PRU=pruDIAG_LI.xml
#DEMO_PRU=test1.xml

# Run simulation

xterm -e roslaunch sim_environment.launch world_file:=\"AUTOGEN_DISB1_diago.world\" &

sleep 3

# Run PRU enabled modules

xterm -hold -e roslaunch diago_all.launch include_robot_navigation:=true pru_enabled:=true PRU:="$DEMO_PRU"  &

sleep 10

# Send the goal from KB - NOT NEEDED!!!

#rostopic pub  /diago/t12_goals_set shared/AllGoals "header:
#  seq: 0
#  stamp:
#    secs: 0
#    nsecs: 0
#  frame_id: '' 
#mode: ''
#goals:
#- {loc: 'printer', kind: 'go', param: '', value: 0.0, duration: 0.0}"  --once
 

sleep 1

xterm -e rostopic echo /diago/pnp/currentActivePlaces &



