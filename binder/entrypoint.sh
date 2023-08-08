#!/bin/bash
source /home/Multiverse/giskard_ws/devel/setup.bash

roscore &
roslaunch --wait rvizweb rvizweb.launch &

# Use xvfb virtual display when there is no display connected.
if [ -n "$DISPLAY" ]; then
    exec "$@"
else
    xvfb-run exec "$@"
fi
