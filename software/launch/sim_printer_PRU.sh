#!/bin/bash

# Run simulated printer demo directly loading the PNP

xterm -e "roslaunch PRUplanner2.launch" &

sleep 15

# Send the goal to KB

rostopic pub  /diago/t12_goals_set shared/AllGoals "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '' 
mode: ''
goals:
- {loc: 'printer', kind: 'go', param: '', value: 0.0, duration: 0.0}"  --once
 

sleep 1

xterm -e "rostopic echo /diago/pnp/currentActivePlaces" &



