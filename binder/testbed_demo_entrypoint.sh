#!/bin/bash
source /home/Multiverse/giskard_ws/devel/setup.bash

multiverse_server &
roscore & 
roslaunch --wait rvizweb rvizweb.launch config_file:=/home/binder/testbed_demo_rvizweb_config.json &
roslaunch --wait tiago_dual_in_testbed demo.launch &
jupyter lab workspaces import /home/binder/testbed_demo.jupyterlab-workspace
exec "$@"