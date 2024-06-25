#!/bin/sh

# launch turtlebot_world.launch to deploy turtlebot environment
xterm -e "cd $(pwd)/../..;
source /opt/ros/foxy/setup.bash 
source install/setup.bash; "
