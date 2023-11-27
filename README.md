# Multiverse-Docker

This repository demonstrates the robotics task execution in Docker environment.

## Prerequisites `https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository`

Check your docker-compose and docker-py versions:

```bash
$ docker -v
Docker version 24.0.7, build afdd53
$ docker compose version
Docker Compose version v2.21.0
```

### 1. Setup the docker

```bash
sudo ./setup_nvidia_docker.bash
./setup_images.bash
docker compose pull
```
