#!/bin/bash

source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 launch slam_toolbox online_async_launch.py params_file:=f1tenth_online_async.yaml
