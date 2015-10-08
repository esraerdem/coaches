================================================================================
                         COACHES Software Environment
                     Luca Iocchi (iocchi@dis.uniroma1.it)
                      Sapienza University of Rome, Italy
================================================================================

The 'coaches/software' folder contains source code, libraries and binary code 
divided in the following sub-folders:

 - src       contains (non-ROS) source code maintained in the coaches repository
 - ros       contains ros modules
 - GUI       contains GUI software
 - external  contains external software not maintained in the coaches repository
 - bin       contains executable files
 - include   contains include files 
 - lib       contains libraries


=== Download the COACHES software environment ===

COACHES software is maintained in a GIT repository at greyc.fr

Download:

  $ git clone https://<username>@forge.greyc.fr/git/coaches
  
If you want to avoid to enter your password at any time, you can apply 
the following settings:

  $ git config --global credential.helper cache
  # Set git to use the credential memory cache

  $ git config --global credential.helper 'cache --timeout=3600'
  # Set the cache to timeout after 1 hour (setting is in seconds)



=== Initialize the COACHES software environment ===

In order to initialize the COACHES software environment for your machine,
you need to run the following steps:

1. Set the environment variable COACHES_HOME in your ~/.bashrc file
   with the full path of the coaches/src directory

   export COACHES_HOME=<PATH>/coaches/software

2. open a new terminal (so that the above setting is applied) and 
   run the following commands

  $ cd $COACHES_HOME
  $ bin/coaches_init.bash

It will ask for sudo password in order to install or update required Linux packages.

If everything goes well, you will see at the end the message 

  Hello COACHES developers!!!

If there are some errors about not found packages (e.g. pnp_ros), you can go on.
Some other packages will be installed in a later step.


=== Set up the COACHES software environment ===

For every terminal in which you want to run or compile COACHES software, 
you need to set up some environment variables with the following command

  $ cd $COACHES_HOME
  $ source bin/coaches_setup.bash

Note:
  If you want variables to be automatically set whenever you open a terminal,
  add 'source $COACHES_HOME/bin/coaches_setup.bash' in your .bashrc file 
  (after the definition of COACHES_HOME).
  Check that there are no conflicts with other projects in your machine 
  (e.g., if you use other ROS catkin workspaces, it may give problems).


=== Update the COACHES software environment ===

After set up (as in the previous section), use:

  $ coaches_update.bash


=== Build the COACHES software environment ===

After set up (as in the previous section), use:

  $ coaches_make.bash

If you get errors here, first time try to run the command again (sometimes catkin
has problems when compiling multiple packages).
If the error is persistent, try to solve it and/or write to iocchi@dis.uniroma1.it.


=== Test the COACHES software environment ===

After set-up and build of the environment.

1. Hello COACHES developers

  $ rosrun hello_coaches_developers hello

You will see a print out 'Hello COACHES developers!!!'

2. Stage environment

  $ roscd stage_environments/scripts/
  $ ./start_simulation.py Rive1 diago 10 23 0 amcl move_base keyboard rviz

You can move the simulated robot with keys WASD when you focus the window in which
you read: "Reading from the keyboard..."
Otherwise you can try to send move_base goal commands in Rviz.

When you want to close the simulation use the command

  $ ./quit.sh


You can also use only the command
  $ ./start_simulation.py 
and select a proper combination in the GUI  



Known problems: 

1. sometimes the very first execution when you turn on the machine does not work.
Solution: retry or try to launch 'roscore' manually before starting the simulation

2. sometimes Rviz shows a black window. 
Solution: re-launch Rviz with the command
  $ rosrun rviz rviz -d `rospack find stage_environments`/config/diago/rviz/diago.rviz
  



