# Multiverse-Docker

1. Setup the docker.

```bash
sudo ./setup_nvidia_docker.bash
./setup_images.bash
docker compose pull
```

2. Run the cluster.

```bash
docker compose up ros-core-service
docker compose up multiverse-server-service
docker compose up multiverse-client-mujoco-service
docker compose up multiverse-client-ros-service
docker compose up giskard-service
./run_rviz.bash
```
