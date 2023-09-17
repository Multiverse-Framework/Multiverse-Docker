#!/bin/bash

xhost +local:root
docker compose up rviz-service

# cleanup after rviz-service has been closed
xhost -local:root