#!/bin/bash

# Launch first terminal
gnome-terminal -- bash -c "run_SLAM.sh; exec bash"

# Wait 1 second
sleep 1

# Launch second terminal
gnome-terminal -- bash -c "rviz2; exec bash"

# Wait 1 second
sleep 1

# Launch third terminal
gnome-terminal -- bash -c "ls; exec bash"
