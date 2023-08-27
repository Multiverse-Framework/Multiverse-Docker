ARG ROS_DISTRO
FROM multiverseframework/giskard:${ROS_DISTRO}

# Zero interaction while installing or upgrading the system via apt, see https://bobcares.com/blog/debian_frontendnoninteractive-docker/
ARG DEBIAN_FRONTEND=noninteractive 

# Copy packages
COPY pycram /home/Multiverse/cram_ws/src/pycram
COPY kdl_ik_services /home/Multiverse/cram_ws/src/kdl_ik_services
WORKDIR /home/Multiverse/cram_ws

# Create symlink for python3 because of some complainings...
RUN ln -sf /usr/bin/python3.8 /usr/bin/python3 

# Install prerequisites
RUN pip3 install psutil
RUN pip3 install -r src/pycram/requirements.txt
RUN pip3 install -r src/pycram/src/neem_interface_python/requirements.txt

# Build catkin workspace
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && source /home/Multiverse/giskard_ws/devel/setup.bash && catkin build

# Create entrypoint
SHELL ["/bin/bash", "-c"]
COPY ./entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash