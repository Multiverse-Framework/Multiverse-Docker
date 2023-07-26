# Multiverse-Docker

1. Setup the docker.

```bash
sudo ./setup_nvidia_docker.bash
./setup_images.bash
```

2. Run the cluster.

```bash
docker compose up ros-core-service
docker compose up multiverse-server-service
./run_multiverse_client_mujoco.bash
./run_multiverse_client_ros.bash
```
