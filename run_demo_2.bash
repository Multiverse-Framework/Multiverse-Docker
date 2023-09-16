#!/bin/bash
xhost +local:root
docker compose exec -it giskard-container source /home/Multiverse/multiverse_ws/devel/setup.bash && rosrun tiago_dual_in_apartment box_unpacking.launch --spawn_object=spoon 

# cleanup after demo-service has been closed
xhost -local:root