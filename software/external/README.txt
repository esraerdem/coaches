This folder contains external software that is needed in the COACHES project, but it is not maintained in the COACHES repository.

This software is installed and updated from external repositories using the command

  $ ./update.bash

The update script also put correct symbolic links into the catkin workspace so that the external ROS packages in this folder are compiled by catkin_make in the catkin workspace.

Building external software is included in the 'coaches_make.bash' script.

Details on how to do it manually:

* PetriNetPlan
  - follow instructions to compile PNP in the PNP/README.txt 
  - set PNP_INCLUDE and PNP_LIB environment variables in the terminal 
    (permanently in the .bashrc file or using the setup.bash file in coaches/src dir)
  
* Other modules
  - run catkin_make from the catkin workspace


