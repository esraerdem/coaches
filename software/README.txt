================================================================================
                         COACHES Software Environment
                     Luca Iocchi (iocchi@dis.uniroma1.it)
                      Sapienza University of Rome, Italy
================================================================================

The 'coaches/software' folder contains source code, libraries and binary code 
divided in the following sub-folders:

 - src       contains (non-ROS) source code maintained in the coaches repository
 - ros       contains ros modules
 - external  contains external software not maintained in the coaches repository
 - bin       contains executable files
 - include   contains include files 
 - lib       contains libraries



=== Initialize the COACHES software environment ===

In order to initialize the COACHES software environment for your machine,
you need to run the following steps:

1. Set the environment variable COACHES_HOME in your ~/.bashrc file
   with the full path of the coaches/src directory

   export COACHES_HOME=<PATH>/coaches/software

2. run the following command from the 'software' directory

  $ cd $COACHES_HOME
  $ bin/coaches_init.bash

It will ask for sudo password in order to install or update required Linux packages.

If everything goes well, you will see at the end the message 

  Hello COACHES developers!!!

If you get errors, try to solve them and/or write to iocchi@dis.uniroma1.it


=== Set up the COACHES software environment ===

For every terminal in which you want to run or compile COACHES software, 
you need to set up some environment variables with the following command

  $ source bin/coaches_setup.bash

Note:
You can add 'source <PATH>/coaches/src/bin/setup.bash' in your .bashrc file 
  if you want variables to be automatically set whener you open a terminal.
  Check that there are no conflicts with other projects in your machine 
  (e.g., if you use other ROS catkin workspaces, it may give problems).


=== Update the COACHES software environment ===

After set up (as in the previous section), use:

  $ coaches_update.bash


=== Build the COACHES software environment ===

After set up (as in the previous section), use:

  $ coaches_make.bash



  



