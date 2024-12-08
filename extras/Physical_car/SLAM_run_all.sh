#!/bin/sh

# Run teleop

xterm -e "./start_car.sh" &

sleep 2.5

#Run Pitstop

xterm -e "./run_teleop_joy.sh" &

sleep 2.0

# Run SLAM Toolbox

xterm -e "./run_slam_toolbox.sh"




