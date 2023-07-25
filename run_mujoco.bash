#!/bin/bash
xhost +local:root
docker compose up mujoco-ws-service

# cleanup after mujoco-ws-service has been closed
xhost -local:root