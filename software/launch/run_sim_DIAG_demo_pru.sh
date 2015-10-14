#!/bin/bash

# Run simulated printer demo directly loading the PNP

DEMO_PRU=pruDIAG.xml
#DEMO_PRU=pruDIAG_LI.xml
#DEMO_PRU=test1.xml

#xterm -e "roslaunch PRUplanner2.launch   PRU:=\"$DEMO_PRU\" " &
xterm -e "roslaunch sim_diago_DIAG.launch pru_enabled:=true PRU:=\"$DEMO_PRU\" " &

sleep 15

# Send the goal from KB

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

xterm -e "rostopic echo /diago/pnp/currentActivePlaces" &



