#!/bin/bash

# Launch first terminal
gnome-terminal -- bash -c "./setup_ethernet.sh; exec bash"

# Wait .5 second
sleep 0.5

# Launch second terminal
gnome-terminal -- bash -c "./start_car.sh; exec bash"


# Launch third terminal

gnome-terminal -- bash -c "./run_slam_toolbox.sh; exec bash"

# Launch RViz2

gnome-terminal --bash -c "rviz2; exec bash"
