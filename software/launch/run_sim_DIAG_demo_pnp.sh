#!/bin/bash

#DEMO_PNP=PrinterAssistance
DEMO_PNP=BenchAssistance
#DEMO_PNP=PersonalizedHelp

# Run simulation

xterm -e roslaunch sim_environment.launch world_file:=\"AUTOGEN_DISB1_diago.world\" &

sleep 3

# Run PNP enabled modules

xterm -e roslaunch diago_all.launch pru_enabled:=false &

sleep 10

rostopic pub /diago/planToExec std_msgs/String "data: '$DEMO_PNP'" --once

sleep 1

xterm -e rostopic echo /diago/pnp/currentActivePlaces &

sleep 1

xterm -e python ../GUI/QAGUI/QAGUI.py &



