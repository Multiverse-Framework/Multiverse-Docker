#!/bin/bash
xhost +local:docker
docker compose -f $(dirname $0)/docker-compose.yml up
xhost -local:docker