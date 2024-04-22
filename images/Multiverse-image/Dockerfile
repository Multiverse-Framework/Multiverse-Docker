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
    git-lfs \
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
    pybind11 \
    urdf_parser_py \
    mujoco==3.1.4

# Instal bash kernel
RUN pip3 install panel jupyter-server bash_kernel
RUN python3 -m bash_kernel.install

# Copy packages
COPY Multiverse /home/Multiverse
WORKDIR /home/Multiverse

ARG MULTIVERSE_DIR=/home/Multiverse/multiverse
ARG BIN_DIR=$MULTIVERSE_DIR/bin
ARG EXT_DIR=$MULTIVERSE_DIR/external
ARG BUILD_DIR=$MULTIVERSE_DIR/build
ARG SRC_DIR=$MULTIVERSE_DIR/src
ARG LIB_DIR=$MULTIVERSE_DIR/lib

RUN mkdir -p $BIN_DIR
RUN mkdir -p $BUILD_DIR

# Build blender
ARG BLENDER_BUILD_DIR=$BUILD_DIR/blender
ARG BLENDER_EXT_DIR=$EXT_DIR/blender-git

RUN mkdir -p $BLENDER_BUILD_DIR
RUN cd $BLENDER_EXT_DIR/blender && ./build_files/utils/make_update.py --use-linux-libraries

RUN sed -i \
    -e 's/g->bitmap\[i\] = blf_glyph_gamma(glyph->bitmap.buffer\[i\] \* scale);/g->bitmap[i] = blf_glyph_gamma(char(glyph->bitmap.buffer[i] * scale));/' \
    "$BLENDER_EXT_DIR/blender/source/blender/blenfont/intern/blf_glyph.cc"

RUN sed -i \
    -e 's/metrics->descender = metrics->ascender - metrics->units_per_EM;/metrics->descender = short(metrics->ascender - metrics->units_per_EM);/' \
    "$BLENDER_EXT_DIR/blender/source/blender/blenfont/intern/blf_font.cc"

RUN sed -i \
    -e 's/direction += 1;/direction = short(direction + 1);/' \
    -e 's/direction -= 1;/direction = short(direction - 1);/' \
    "$BLENDER_EXT_DIR/blender/source/blender/editors/space_view3d/view3d_navigate_walk.cc"

RUN sed -i \
    -e 's/ = const_cast<const BMLoop \*(\*)\[3\]>(em->looptris);/;memcpy(corner_tris, em->looptris, sizeof(*em->looptris));/' \
    "$BLENDER_EXT_DIR/blender/source/blender/blenkernel/intern/bvhutils.cc"

RUN sed -i \
    -e 's/mesh2tangent->looptris = const_cast<const BMLoop \*(\*)\[3\]>(em->looptris);/memcpy(mesh2tangent->looptris, em->looptris, sizeof(*em->looptris));/' \
    "$BLENDER_EXT_DIR/blender/source/blender/blenkernel/intern/editmesh_tangent.cc"

RUN sed -i \
    -e 's/bmcb_data.looptris = const_cast<const BMLoop \*(\*)\[3\]>(bmtree->looptris);/const BMLoop *tmp_looptris[3] = {nullptr, nullptr, nullptr}; bmcb_data.looptris = \&tmp_looptris; memcpy(bmcb_data.looptris, bmtree->looptris, sizeof(*bmtree->looptris));/' \
    -e 's/bmcb_data->looptris = const_cast<const BMLoop \*(\*)\[3\]>(bmtree->looptris);/const BMLoop *tmp_looptris[3] = {nullptr, nullptr, nullptr}; bmcb_data->looptris = \&tmp_looptris; memcpy(bmcb_data->looptris, bmtree->looptris, sizeof(*bmtree->looptris));/' \
    "$BLENDER_EXT_DIR/blender/source/blender/blenkernel/intern/editmesh_bvh.cc"

RUN cd $BLENDER_BUILD_DIR && cmake -S ../../external/blender-git/blender -B . -Wno-deprecated -Wno-dev && make -j$(nproc) && make install
RUN cd $BLENDER_BUILD_DIR/bin/4.1/python/bin && ./python3.11 -m pip install --upgrade pip build --no-warn-script-location && ./python3.11 -m pip install bpy Pillow --no-warn-script-location
RUN ln -sf $BLENDER_BUILD_DIR/bin/blender $BIN_DIR
RUN ln -sf $BLENDER_BUILD_DIR/bin/4.1/python/bin/python3.11 $BIN_DIR

# Build USD

ARG USD_BUILD_DIR=$BUILD_DIR/USD
ARG USD_EXT_DIR=$EXT_DIR/USD

RUN mkdir -p $USD_BUILD_DIR

RUN python3 $USD_EXT_DIR/build_scripts/build_usd.py $USD_BUILD_DIR
RUN ln -sf $USD_BUILD_DIR/bin/usdview $BIN_DIR
RUN ln -sf $USD_BUILD_DIR/bin/usdGenSchema $BIN_DIR
RUN ln -sf $USD_BUILD_DIR/bin/usdcat $BIN_DIR

# Build MuJoCo

ARG MUJOCO_BUILD_DIR=$BUILD_DIR/mujoco
ARG MUJOCO_EXT_DIR=$EXT_DIR/mujoco

RUN mkdir -p $MUJOCO_BUILD_DIR

RUN cd $MUJOCO_BUILD_DIR && cmake $MUJOCO_EXT_DIR -DCMAKE_INSTALL_PREFIX=$MUJOCO_BUILD_DIR -Wno-deprecated -Wno-dev && cmake --build . && cmake --install .
RUN ln -sf $MUJOCO_BUILD_DIR/bin/simulate $BIN_DIR

# Add PATH and PYTHONPATH
ENV PATH=$PATH:$BIN_DIR
ENV PYTHONPATH=$PYTHONPATH:$LIB_DIR/libstdc++/python:$USD_BUILD_DIR/lib/python

RUN echo "export PATH=$PATH" >> ~/.bashrc
RUN echo "export PYTHONPATH=$PYTHONPATH" >> ~/.bashrc

# Build the software
RUN cmake -S $MULTIVERSE_DIR -B $BUILD_DIR -DCMAKE_INSTALL_PREFIX:PATH=$MULTIVERSE_DIR -DMULTIVERSE_CLIENT_LIBRARY_TYPE=STATIC -DSTDLIB=libstdc++
RUN make -C $BUILD_DIR
RUN cmake --install $BUILD_DIR

# Build the ROS workspace
RUN ./build_multiverse_ws.sh && ./build_multiverse_ws2.sh

RUN apt-get update && apt-get upgrade -y

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