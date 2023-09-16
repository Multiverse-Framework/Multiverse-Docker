# Multiverse Demo on BinderHub

Files for running demo in a single docker container on BinderHub.

## Setup

### Option 1: Run Image Locally (Under root directory `Multiverse-Docker`)

- Run Docker image with X-forwarding

```bash
./binder/run_local.sh
```

- Open Web browser and go to `http://localhost:8888/`

- Force rebuilding image

```bash
docker build ./ -f ./binder/Dockerfile -t multiversedocker:binderhub
```

### Option 2: Run on BinderHub

- [Link to the binderhub](https://binder.intel4coro.de/v2/gh/Multiverse-Framework/Multiverse-Docker.git/main)

## Usage

The default entrypoint will start the Roscore and the following ROS nodes:

- `/ros_board`
- `/rosbridge_websocket`
- `/roswww`
- `/tf2_web_republisher`

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
      roslaunch tiago_dual_in_apartment multiverse_socket.launch
    ```

1. Start ***giskard-service***

    ```bash
    source /home/Multiverse/giskard_ws/devel/setup.bash && \
      roslaunch tiago_dual_control tiago_dual_giskard.launch
    ```

1. Launch ***rviz-service*** (Works only locally)

    ```bash
    source /home/Multiverse/giskard_ws/devel/setup.bash && \
      roslaunch tiago_dual_in_apartment rviz.launch
    ```

1. Update rvizweb config file and run node interactive_marker_proxy (equivalent to ***rviz-service***)

    ```bash
    source /home/Multiverse/giskard_ws/devel/setup.bash && \
      roslaunch rvizweb update_config.launch config_file:=/home/binder/rvizweb-config.json && \
      rosrun interactive_marker_proxy proxy topic_ns:=/eef_control target_frame:=/base_link &
    ```

### All-in-One script

The script that runs all the above services:

```bash
  /home/binder/demo.sh
```

## Files Descriptions

1. ***[Dockerfile](./Dockerfile):*** A Jupyterlab Docker image base on [multiverseframework/giskard:noetic](https://hub.docker.com/r/multiverseframework/giskard).
2. ***[compress_textures.py](./compress_textures.py):*** A Python script to compress the 3d model texture files.
3. ***[entrypoint.sh](./entrypoint.sh):*** Entrypoint of the docker image.
4. ***[iros.jupyterlab-workspace](./iros.jupyterlab-workspace):*** A Custom JupyterLab workspace for iros demo.
5. ***[jupyter-settings.json](./jupyter-settings.json):*** Jupyterlab settings (e.g., Themes, Font size, Default indent and so on).
6. ***[rvizweb-config.json](./rvizweb-config.json):*** Rvizweb configuration file.
7. ***[docker-compose.yml](./docker-compose.yml):*** For testing the docker image locally.
8. ***[run_local.sh](./run_local.sh):*** For testing the docker image locally.
9. ***[demo.sh](./demo.sh):*** Run all services for iros demo.
10. ***[me/](./me):*** A temporar workaround to fix incorrect mesh resource url of Marker Array (Robot Ghost). (At the moment the mesh_resource urls are in such format: `file:///home/Multiverse/giskard_ws/src/giskardpy/tmp/tiago_dual/\*.obj`. Should be replace to `package://giskardpy/tmp/tiago_dual/\*.obj`)
