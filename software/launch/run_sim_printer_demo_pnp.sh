#!/bin/bash

# Run simulated printer demo directly loading the PNP

xterm -e "roslaunch simDIAGPrinter.launch pru_enabled:=false" &

sleep 15

rostopic pub /diago/planToExec std_msgs/String "data: 'BenchPatrol'" --once

sleep 1

xterm -e "rostopic echo /diago/pnp/currentActivePlaces" &



