#!/bin/sh

# launch turtlebot_world.launch to deploy turtlebot environment
xterm -e "cd $(pwd)/../..;
source install/setup.bash;
ros2 launch lab1_pkg lab1_launch.py "
