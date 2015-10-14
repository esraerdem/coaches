#!/bin/bash

#DEMO_PNP=PrinterAssistance
DEMO_PNP=BenchPatrol

# Run simulated printer demo directly loading the PNP

xterm -e "roslaunch sim_diago_DIAG.launch pru_enabled:=false" &

sleep 15

rostopic pub /diago/planToExec std_msgs/String "data: '$DEMO_PNP'" --once

sleep 1

xterm -e "rostopic echo /diago/pnp/currentActivePlaces" &


