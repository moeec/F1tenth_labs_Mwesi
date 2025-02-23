#!/bin/bash

source install/setup.bash

cd ~/f1tenth_ws/src/F1tenth_labs_Mwesi/lab6_ws/pure_pursuit/config

ros2 launch slam_toolbox online_async_launch.py params_file:=f1tenth_online_async.yaml
