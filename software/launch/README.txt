================================
  How to Launch COACHES demos
================================

Only the first time, you need to create the simulated worlds with:

  $ ./create_world.sh 

To quit the simulation, you can just close either the Stage window or the Control GUI.



1. Simulation in Rive1 map (current version with mockups):

  $ roslaunch simWithMock.launch 



2. Simulation printer demo at DIAG (load PNP directly)

  $ ./run_printer_demo.sh



3. Real DIAGO printer demo at DIAG (load PNP directly)

  $ ./run_diago_printer_demo.sh


4. Simulaion printer demo at DIAG (from PRU)

  $ ./sim_printer_PRU.sh




To send a goal to PRUplanner

To set a condition for PNP

rostopic pub /diago/PNPConditionEvent std_msgs/String "data: 'CONDITION'" --once

