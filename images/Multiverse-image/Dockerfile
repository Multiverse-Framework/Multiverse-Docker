FROM ubuntu:20.04

# Zero interaction while installing or upgrading the system via apt, see https://bobcares.com/blog/debian_frontendnoninteractive-docker/
ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y software-properties-common curl

# Set locale (https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
RUN apt-get install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN LANG=en_US.UTF-8

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN add-apt-repository ppa:ubuntu-toolchain-r/test
RUN apt-get update && apt-get install -y ros-noetic-desktop-full python3-catkin-tools

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN add-apt-repository universe
RUN apt-get update && apt-get install -y ros-foxy-desktop ros-dev-tools

# Install prerequisites
RUN apt-get update && apt-get install -y \
    python-is-python3 \
    python3-catkin-tools \
    libglfw3 \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libxt-dev \
    git \
    libjsoncpp-dev \
    libzmqpp-dev \
    build-essential \
    subversion \
    cmake \
    libx11-dev \
    libxxf86vm-dev \
    libxcursor-dev \
    libxi-dev \
    libxrandr-dev \
    libxinerama-dev \
    libegl-dev \
    libwayland-dev \
    wayland-protocols \
    libxkbcommon-dev \
    libdbus-1-dev \
    linux-libc-dev \
    pybind11-dev

# Install and link python3.8
RUN apt-get install -y python3.8-dev python3.8-venv && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 100

# Install and link gcc-11
RUN apt-get install -y gcc-11 && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100

# Install and link clang-11
RUN apt-get install -y clang-11 libc++-11-dev libstdc++-11-dev libc++abi-11-dev llvm-11-dev && update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang-11 100

# Install rosdep
RUN apt-get install -y python3-rosdep && rosdep init && rosdep fix-permissions && rosdep update

# Install pip
RUN apt-get install -y python3-pip && pip install --upgrade pip build

RUN pip install \
    pyside6 \
    pyopengl \
    wheel \
    cython \
    owlready2 \
    markupsafe==2.0.1 \
    jinja2 \
    pybind11

# Instal bash kernel
RUN pip3 install panel jupyter-server bash_kernel
RUN python3 -m bash_kernel.install

# Copy packages
COPY Multiverse /home/Multiverse
WORKDIR /home/Multiverse

# Build the dependencies
RUN ./build_third_parties.sh

# Build the software
RUN ./build_multiverse.sh

# Build the ROS workspace
RUN ./build_multiverse_ws.sh && ./build_multiverse_ws2.sh

ENV PATH=${PATH}:/home/Multiverse/multiverse/bin
ENV PYTHONPATH=${PYTHONPATH}:/home/Multiverse/multiverse/build/USD/lib/python:/home/Multiverse/multiverse/lib/libstdc++/python

RUN RUN apt-get update && apt-get upgrade -y 

# Install additional packages for ROS1
RUN apt-get install -y \
    ros-noetic-xacro \
    ros-noetic-rviz \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-rqt-robot-steering \
    ros-noetic-rqt-joint-trajectory-controller \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-position-controllers \
    ros-noetic-velocity-controllers \
    ros-noetic-effort-controllers \
    ros-noetic-gripper-action-controller \
    ros-noetic-dwa-local-planner \
    ros-noetic-cob-gazebo-objects \
    ros-noetic-map-server \
    ros-noetic-move-base

# Install additional packages for ROS2
RUN apt-get install -y \
    ros-foxy-xacro \
    ros-foxy-rviz2 \
    ros-foxy-joint-trajectory-controller \
    ros-foxy-rqt-robot-steering \
    ros-foxy-joint-state-controller \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-position-controllers \
    ros-foxy-velocity-controllers \
    ros-foxy-effort-controllers \
    ros-foxy-gripper-controllers