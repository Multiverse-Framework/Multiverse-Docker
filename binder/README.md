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
docker compose build
```

### Option 2: Run on BinderHub

- [Link to the binderhub](https://binder.intel4coro.de/v2/gh/Multiverse-Framework/Multiverse-Docker/main)

## Usage

The default entrypoint will start the Roscore and the following ROS nodes:

- `/ros_board`
- `/rosbridge_websocket`
- `/roswww`
- `/tf2_web_republisher`

## Files Descriptions

1. ***[Dockerfile](./Dockerfile):*** A Jupyterlab Docker image base on [multiverseframework/giskard:noetic](https://hub.docker.com/r/multiverseframework/giskard).
2. ***[compress_textures.py](./compress_textures.py):*** A Python script to compress the 3d model texture files.
3. ***[entrypoint.sh](./entrypoint.sh):*** Entrypoint of the docker image.
4. ***[jupyter-settings.json](./jupyter-settings.json):*** Jupyterlab settings (e.g., Themes, Font size, Default indent and so on).
5. ***[rvizweb-config.json](./rvizweb-config.json):*** Rvizweb configuration file.
6. ***[docker-compose.yml](./docker-compose.yml):*** For testing the docker image locally.
7. ***[run_local.sh](./run_local.sh):*** For testing the docker image locally.
8. ***[me/](./me):*** A temporar workaround to fix incorrect mesh resource url of Marker Array (Robot Ghost). (At the moment the mesh_resource urls are in such format: `file:///home/Multiverse/giskard_ws/src/giskardpy/tmp/tiago_dual/\*.obj`. Should be replace to `package://giskardpy/tmp/tiago_dual/\*.obj`)
