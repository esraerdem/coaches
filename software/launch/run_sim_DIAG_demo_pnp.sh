#!/bin/bash

#DEMO_PNP=PrinterAssistance
#DEMO_PNP=BenchAssistance
DEMO_PNP=PersonalizedHelp

# Run simulation

xterm -e roslaunch sim_environment.launch world_file:=\"AUTOGEN_DISB1_diago.world\" &

#`(rospack find diago_apps)`/script/run_navigation.py diago AUTOGEN_DISB1_diago.world SIM 2.6 13 0 180

sleep 3

# Run PNP enabled modules

xterm -e roslaunch diago_all.launch include_robot_navigation:=true pru_enabled:=false initial_pose_x:=2.6 initial_pose_y:=13 initial_pose_a:=3.14 &

sleep 10

rostopic pub /diago/planToExec std_msgs/String "data: '$DEMO_PNP'" --once

sleep 1

xterm -e rostopic echo /diago/pnp/currentActivePlaces &

sleep 1

xterm -e python ../GUI/QAGUI/QAGUI.py &



