# Multiverse-Docker

This repository demonstrates the robotics task execution in Docker environment. For more details please visit [Multiverse-Framework](https://github.com/Multiverse-Framework/Multiverse/tree/ICRA-2024), [Multiverse-Parser](https://github.com/Multiverse-Framework/Multiverse/tree/ICRA-2024/multiverse/modules/multiverse_parser) and [Multiverse-Knowledge](https://github.com/Multiverse-Framework/Multiverse/tree/ICRA-2024/multiverse/modules/multiverse_knowledge).

## Prerequisites `https://docs.docker.com/desktop/install/ubuntu/`

Check your docker-compose and docker-py versions:

```bash
$ docker -v
Docker version 24.0.5, build ced0996
$ docker compose version
Docker Compose version v2.20.2
```

### 1. Setup the docker and Unreal Engine packages

```bash
sudo ./setup_nvidia_docker.bash
./setup_images.bash
./setup_packages.bash
docker compose pull
```

### 2. Run the cluster

Open 6 terminals, each runs one command below
```bash
docker compose up ros-core-service
docker compose up multiverse-server-service
docker compose up multiverse-client-mujoco-service
docker compose up multiverse-client-ros-service
docker compose up giskard-service
./run_rviz.bash
```

### 3. Run the demo (either `demo_1` or `demo_2`)

Open a new terminal and get inside the docker container `giskard-container`

```bash
docker exec -it giskard-container bash
```

Run either

```bash
source /home/Multiverse/giskard_ws/devel/setup.bash && rosrun tiago_dual_in_apartment box_unpacking.py --spawn_object=milk_box 
```

or

```bash
source /home/Multiverse/giskard_ws/devel/setup.bash && rosrun tiago_dual_in_apartment box_unpacking.py --spawn_object=spoon 
```

To visualize the simulation with Unreal Engine, open a new terminal and run either

```bash
./UE-packages/Demo1/MultiverseDemo.sh # For milk_box demo
```

or

```bash
./UE-packages/Demo2/MultiverseDemo.sh # For spoon demo
```

The result should look like this:

![Result](https://github.com/Multiverse-Framework/Multiverse-Docker/assets/64316740/a7aa9d00-5fa5-4bbf-a376-67ad884fdd1a)


### 4. Connect to docker container via ROS

```bash
export ROS_MASTER_URI=http://192.168.75.2:11311
export ROS_IP=$(hostname -I | awk '{print $1}') #your IP
rostopic list # It will show all ROS topic in the docker environment
```
