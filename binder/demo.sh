#!/bin/bash

multiverse_server &

source /home/Multiverse/multiverse_ws/devel/setup.bash && \
  roslaunch tiago_dual_in_testbed demo.launch headless:=true &

source /home/Multiverse/giskard_ws/devel/setup.bash && \
  roslaunch rvizweb update_config.launch config_file:=/home/binder/rvizweb-config.json && \
  rosrun interactive_marker_proxy proxy topic_ns:=/eef_control target_frame:=/base_link &

wait