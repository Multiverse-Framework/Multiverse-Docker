ARG ROS_DISTRO
FROM multiverseframework/multiverse-demos:ubuntu-20.04

# Zero interaction while installing or upgrading the system via apt, see https://bobcares.com/blog/debian_frontendnoninteractive-docker/
ARG DEBIAN_FRONTEND=noninteractive

RUN git remote set-url origin https://github.com/Multiverse-Framework/Multiverse.git && git checkout main && git pull origin main
RUN cd /home/Multiverse/multiverse_ws/src/multiverse_world && git remote set-url origin https://github.com/Multiverse-Framework/Multiverse-World.git && git checkout main && git pull origin main
RUN cd /home/Multiverse/multiverse_ws/src/multiverse_robots && git remote set-url origin https://github.com/Multiverse-Framework/Multiverse-Robots.git && git checkout main && git pull origin main
RUN cd /home/Multiverse/multiverse_ws/src/multiverse_objects && git remote set-url origin https://github.com/Multiverse-Framework/Multiverse-Objects.git && git checkout main && git pull origin main
RUN cd /home/Multiverse/multiverse_ws/src/multiverse_demos && git remote set-url origin https://github.com/Multiverse-Framework/Multiverse-Demos.git && git checkout main && git pull origin main

# Copy packages
COPY giskard_msgs /home/Multiverse/giskard_ws/src/giskard_msgs
COPY giskardpy /home/Multiverse/giskard_ws/src/giskardpy
WORKDIR /home/Multiverse/giskard_ws

# Install prerequisites
RUN pip3 install -r src/giskardpy/requirements.txt

# Run script build_better_pybullet.sh, see https://github.com/SemRoCo/giskardpy#fast-custom-bullet-bindings
RUN cd /home/Multiverse/giskard_ws/src/giskardpy/scripts && ./build_better_pybullet.sh /home
ENV PYTHONPATH=${PYTHONPATH}:/home/bullet3/build_cmake/better_python:/home/bullet3/examples/pybullet

# Install qpSWIFT
RUN cd /home && git clone https://github.com/qpSWIFT/qpSWIFT.git
RUN cd /home/qpSWIFT/python && sudo python3 setup.py install

# Build catkin workspace
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && source /home/Multiverse/multiverse_ws/devel/setup.bash && catkin build

# Create entrypoint
SHELL ["/bin/bash", "-c"]
COPY ./entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash