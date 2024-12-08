#!/bin/bash

source /opt/ros/foxy/setup.bash
source install/setup.bash

cd ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config

ros2 launch slam_toolbox online_async_launch.py params_file:=f1tenth_online_async.yaml
