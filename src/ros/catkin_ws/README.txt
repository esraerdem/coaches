This folder contains Catkin ROS nodes.

*IMPORTANT*

You have to initialize your Catkin workspace the very first time you use it, following these instructions:

1. Initialize the workspace from the src folder:

  $ cd src
  $ catkin_init_workspace
  $ cd ..


2. Compile the workspace

  $ catkin_make


3. Set-up your environment variables (*)

  $ source devel/setup.bash

(*) Setting environment variables can be done permanently by adding the source command 
in your ~/.bashrc file. [Preferred option if you do not have other catkin workspaces in your machine]
Otherwise, it must be run manually when you want to use this workspace (just once for each terminal). [Needed if you want to manage multiple workspaces.]


4. Run roscore in another terminal

  $ roscore


5. Run hello in this terminal (or in any terminal in which you have set-up environment variables as described in step 3).

  $ rosrun hello_coaches_developers hello


6. Read the hello message!










