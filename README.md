# Multiverse-Docker

1. Setup the docker

```bash
sudo ./setup_nvidia_docker.bash
./setup_images.bash
docker compose pull
```

2. Run the cluster

```bash
docker compose up ros-core-service
docker compose up multiverse-server-service
docker compose up multiverse-client-mujoco-service
docker compose up multiverse-client-ros-service
docker compose up giskard-service
./run_rviz.bash
```

3. Run CRAM as standalone

```bash
docker compose up ros-core-service
docker compose up cram-service
./run_demo.bash
```

4. Connect to docker container via ROS

```bash
export ROS_MASTER_URI=http://192.168.75.2:11311
export ROS_IP=$(hostname -I | awk '{print $1}') #your IP
```
