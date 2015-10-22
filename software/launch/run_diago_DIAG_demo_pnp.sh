#!/bin/bash

#DEMO_PNP=PrinterAssistance
#DEMO_PNP=BenchAssistance
DEMO_PNP=PersonalizedHelp

# Run navigation modules for real robot

`(rospack find diago_apps)`/script/run_navigation.py diago DISB1 REAL 2.2 11.8 180

sleep 5

# Run PNP enabled modules

xterm -e roslaunch diago_all.launch pru_enabled:=false initial_pose_x:=2.2 initial_pose_y:=11.8 initial_pose_a:=3.14 &

xterm -e ./joyevent.py  &

sleep 15

rostopic pub /diago/planToExec std_msgs/String "data: '$DEMO_PNP'" --once

sleep 1

xterm -e rostopic echo /diago/pnp/currentActivePlaces &

sleep 1

#xterm -e python ../GUI/QAGUI/QAGUI.py &



