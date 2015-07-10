#!/bin/bash

`(rospack find diago_apps)`/script/run_navigation.py diago DISB1 REAL 2.2 11.8 180

sleep 5

xterm -e "roslaunch diagoDIAGPrinter.launch" &


sleep 15

xterm -e ./joyevent.py  &

rostopic pub /diago/planToExec std_msgs/String "data: 'PrinterAssistance'" --once

sleep 1

xterm -e "rostopic echo /diago/pnp/currentActivePlaces" &



