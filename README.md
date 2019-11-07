# 2D-SLAM-Robot-Lab
- Implement SLAM on a robot equipped with LiDAR and GUI controller on computer.

- See details in "2D-SLAM-Robot-Lab.pdf".

## Folder Structures in "Codes/botlab/"
### Directories
- bin/
    - where all built binaries are located
    - you'll be using this directory a lot   
- data/
    - where data needed to run parts of the assignment are located
    - log files and target files for SLAM and exploration are here 
- lcmtypes/
    - where the .lcm type definitions are located
    - the generated types are stored in src/lcmtypes
- lib/
    - where static libraries are saved during the build process
    - you should never need to manually do anything in this directory
- src/
    - where all source code for botlab is located
    - the subdirectories will have a further description of their contents

### Files
- Makefile
    - the root Makefile that launches the recursive build of the botlab code
    - you shouldn't need to edit this file
- log_mbot_sensors.sh
    - a script to log the sensor data needed for SLAM so you can easily create your own log files 
      for testing
- setenv.sh
    - a script for setting environment variables needed for running Vx applications
    - run this script before running botgui in a terminal on your laptop
    - run via `. setenv.sh` -- note the space

## Folder Structures in "Codes/mobilebot/"
- bin/		  	               : Binaries folder
- mobilebot/mobilebot.c/.h     : Main setup and threads
- test_motors/test_motors.c/.h : Program to test motor implementation
- common/mb_controller.c/.h    : Contoller for manual and autonomous nav
- common/mb_defs.h             : Define hardware config
- common/mb_odometry.c/.h	   : Odometry and dead reckoning 
- lcmtypes/                    : lcmtypes for Mobilebot
- java/                        : java build folder for lcmtypes for lcm-spy
- optitrack/                   : program for reading Optitrack data and publishing lcm messages
- setenv.sh                    : sets up java PATH variables for lcm-spy (run with: source setenv.sh)
