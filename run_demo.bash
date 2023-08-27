#!/bin/bash
xhost +local:root
docker compose up demo-service

# cleanup after demo-service has been closed
xhost -local:root