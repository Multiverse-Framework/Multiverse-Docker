# Multiverse Demo on BinderHub

Files for running IROS demo in a single docker container on BinderHub.

## Setup

### Option 1: Run Image Locally (Under root directory "Multiverse-Docker")
- Run Docker image with X-forwarding

  ./binder/run_local.sh

- Open Web browser and go to http://localhost:8888/

- Force rebuilding image

  docker build ./ -f ./binder/Dockerfile -t multiversedocker:binderhub

### Option 2: Run on BinderHub
- [Link to the binderhub](https://binder.intel4coro.de/v2/git/https%3A%2F%2Fgithub.com%2FMultiverse-Framework%2FMultiverse-Docker.git/HEAD)


## Usage
The default entrypoint will start the Roscore and the following ROS nodes:
- /ros_board
- /rosbridge_websocket
- /roswww
- /tf2_web_republisher

The other 4 services in the [../docker-compose.yml](../docker-compose.yml) need to run manually. Open Terminal and run the following commands.

1. Start ***multiverse-server-service***
    ```bash
    source /home/Multiverse/multiverse_ws/devel/setup.bash && \
      roslaunch multiverse_server multiverse_server.launch
    ```

1. Start ***multiverse-client-mujoco-service***
    ```bash
    source /home/Multiverse/multiverse_ws/devel/setup.bash && \
      roslaunch tiago_dual_in_apartment mujoco.launch headless:=true
    ```

1. Start ***multiverse-client-ros-service***
    ```bash
    source /home/Multiverse/multiverse_ws/devel/setup.bash && \
      roslaunch tiago_dual_in_apartment multiverse_client.launch
    ```

1. Start ***giskard-service***
    ```bash
    source /home/Multiverse/giskard_ws/devel/setup.bash && \
      roslaunch tiago_dual_control tiago_dual_giskard.launch
    ```
    
1. Update rvizweb config file (equivalent to ***rviz-service***)
    ```bash
    source /home/Multiverse/giskard_ws/devel/setup.bash && \
      roslaunch rvizweb update_config.launch config_file:=/home/jovyan/repo/binder/rvizweb-config.json
    ```

## Files Descriptions
1. ***[Dockerfile](./Dockerfile):*** A Jupyterlab Docker image base on [multiverseframework/giskard:noetic](https://hub.docker.com/r/multiverseframework/giskard).
1. ***[compress_textures.py](./compress_textures.py):*** A Python script to compress the 3d model texture files.
1. ***[multiverse_params.yaml](./multiverse_params.yaml):*** A fixed config file of `multiverse-server-service`.
1. ***[entrypoint.sh](./entrypoint.sh):*** Entrypoint of the docker image.
1. ***[iros.jupyterlab-workspace](./iros.jupyterlab-workspace):*** Custom JupyterLab workspace for iros demo.
1. ***[app.json](./app.json):*** Configuration of Jupyterlab extensions (e.g, Rvizweb, Rosboard).
1. ***[rvizweb-config.json](./rvizweb-config.json):*** Rvizweb configuration file.
1. ***[docker-compose.yml](./docker-compose.yml):*** For testing the docker image locally.
1. ***[run_local.sh](./run_local.sh):*** For testing the docker image locally.
