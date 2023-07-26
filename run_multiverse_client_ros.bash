#!/bin/bash
xhost +local:root
docker compose up multiverse-client-ros-service

# cleanup after multiverse-client-ros-service has been closed
xhost -local:root