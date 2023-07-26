ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-core

# Zero interaction while installing or upgrading the system via apt, see https://bobcares.com/blog/debian_frontendnoninteractive-docker/
ARG DEBIAN_FRONTEND=noninteractive 

# Copy packages
COPY Multiverse /home/Multiverse
WORKDIR /home/Multiverse

# Install prerequisites
RUN apt-get update && apt install -y \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-rviz \
  ros-${ROS_DISTRO}-controller-manager \
  ros-${ROS_DISTRO}-joint-trajectory-controller \
  ros-${ROS_DISTRO}-rqt-robot-steering \
  ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
  ros-${ROS_DISTRO}-joint-state-controller \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-effort-controllers \
  ros-${ROS_DISTRO}-gripper-action-controller \
  ros-${ROS_DISTRO}-move-base \
  ros-${ROS_DISTRO}-map-server \
  ros-${ROS_DISTRO}-dwa-local-planner

RUN apt install -y \
  python3-pip \
  python3.8-dev \
  python3-catkin-tools \
  python3-rosdep \
  libglfw3 \
  libglfw3-dev \
  doxygen \
  libgl1-mesa-dev \
  libglu1-mesa-dev \
  libxt-dev \
  git \
  libjsoncpp-dev \
  libzmqpp-dev

RUN pip3 install \
  pyside6 \
  pyopengl \
  mujoco-py \
  wheel \
  cython \
  owlready2 \
  jinja2 \
  numpy-stl \
  pybind11

RUN ./build.sh

# Create entrypoint
SHELL ["/bin/bash", "-c"]
COPY ./entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash