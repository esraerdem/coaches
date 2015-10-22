#!/bin/bash

# Run simulated printer demo directly loading the PNP

DEMO_PRU=pruDIAG.xml
#DEMO_PRU=pruDIAG_LI.xml
#DEMO_PRU=test1.xml

# Run navigation modules for real robot

`(rospack find diago_apps)`/script/run_navigation.py diago DISB1 REAL 2.2 11.8 180

sleep 5

# Run PRU enabled modules

xterm -e roslaunch diago_all.launch pru_enabled:=true PRU:="$DEMO_PRU" initial_pose_x:=2.2 initial_pose_y:=11.8 initial_pose_a:=3.14 &

xterm -e ./joyevent.py  &

sleep 15

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



