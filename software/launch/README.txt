================================
  How to Launch COACHES demos
================================

Only the first time, you need to create the simulated worlds with:

  $ ./create_world.sh 


RUNNING THE DEMOS

1. Simulation in Rive1 map (current version with mockups):

  $ roslaunch simWithMock.launch 


2. Simulation demo at DIAG

  $ ./run_sim_DIAG_demo_{pru|pnp}.sh

Using either PRU or PNP as input. 
Edit the .sh file to specify which demo you want to start.


3. Real DIAGO printer demo at DIAG (load PNP directly)

  $ ./run_diago_DIAG_demo_{pru|pnp}.sh

Same as before, but with the real robot.


To quit the simulation, you can just close either the Stage window or the Control GUI.

Otherwise, you can kill all the ROS nodes: 

  $ rosnode kill -a

or all the xterm windows: 

  $ killall -9 xterm



To send a goal to PRUplanner use the following

rostopic pub  /diago/t12_goals_set shared/AllGoals "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '' 
  mode: ''
  goals:
  - {loc: 'printer', kind: 'go', param: '', value: 0.0, duration: 0.0}"  --once



To set a condition for PNP

rostopic pub /diago/PNPConditionEvent std_msgs/String "data: 'CONDITION'" --once


