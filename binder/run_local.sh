#!/bin/bash
xhost +local:docker
docker compose -f ./binder/docker-compose.yml up
xhost -local:docker