#!/bin/bash
xhost +local:root
docker compose up multiverse-client-mujoco-service

# cleanup after multiverse-client-mujoco-service has been closed
xhost -local:root