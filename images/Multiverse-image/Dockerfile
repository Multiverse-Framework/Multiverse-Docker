ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-core

# Zero interaction while installing or upgrading the system via apt, see https://bobcares.com/blog/debian_frontendnoninteractive-docker/
ARG DEBIAN_FRONTEND=noninteractive 

# Copy packages
COPY Multiverse /home/Multiverse
WORKDIR /home/Multiverse

# Install prerequisites
RUN apt-get update && apt-get install -y \
  python3-pip \
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
  libzmqpp-dev \
  build-essential \
  software-properties-common \
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
RUN apt-get install -y python3.8-dev && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 100

# Install and link gcc-11
RUN add-apt-repository ppa:ubuntu-toolchain-r/test && apt install -y gcc-11 && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100

# Install and link clang-11
RUN apt-get install -y clang-11 libc++-11-dev libstdc++-11-dev libc++abi-11-dev llvm-11-dev && update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang-11 100

RUN pip3 install \
  pyside6 \
  pyopengl \
  wheel \
  cython \
  owlready2 \
  jinja2 \
  pybind11

RUN ./build_third_parties.sh

RUN ./build_multiverse.sh

RUN rosdep init && rosdep fix-permissions && ./build_multiverse_ws.sh

# Create entrypoint
SHELL ["/bin/bash", "-c"]
COPY ./entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash