#!/bin/bash
source /home/Multiverse/giskard_ws/devel/setup.bash

roscore & 
roslaunch --wait rvizweb rvizweb.launch &
roslaunch --wait rosboard rosboard.launch &
jupyter lab workspaces import /home/${NB_USER}/repo/binder/iros.jupyterlab-workspace
exec "$@"
