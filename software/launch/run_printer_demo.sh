#!/bin/bash

xterm -e "roslaunch simDIAGPrinter.launch" &

sleep 15

rostopic pub /diago/planToExec std_msgs/String "data: 'PrinterAssistance'" --once

sleep 1

xterm -e "rostopic echo /diago/pnp/currentActivePlaces" &



