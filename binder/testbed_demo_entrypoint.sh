#!/bin/bash
source /home/Multiverse/giskard_ws/devel/setup.bash

roscore & 
roslaunch --wait rvizweb rvizweb.launch &
jupyter lab workspaces import /home/binder/main.jupyterlab-workspace
exec "$@"