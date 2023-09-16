ARG ROS_DISTRO
FROM multiverseframework/multiverse:icra-2024

# Zero interaction while installing or upgrading the system via apt, see https://bobcares.com/blog/debian_frontendnoninteractive-docker/
ARG DEBIAN_FRONTEND=noninteractive 

# Copy packages
COPY multiverse_objects /home/Multiverse/multiverse_ws/src/multiverse_objects
COPY multiverse_robots /home/Multiverse/multiverse_ws/src/multiverse_robots
COPY multiverse_world /home/Multiverse/multiverse_ws/src/multiverse_world
COPY multiverse_demos /home/Multiverse/multiverse_ws/src/multiverse_demos

# Install prerequisites
RUN apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-rviz \
  ros-${ROS_DISTRO}-joint-trajectory-controller \
  ros-${ROS_DISTRO}-rqt-robot-steering \
  ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
  ros-${ROS_DISTRO}-joint-state-controller \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-effort-controllers \
  ros-${ROS_DISTRO}-gripper-action-controller \
  ros-${ROS_DISTRO}-dwa-local-planner

RUN cd multiverse_ws && rosdep install --from-paths src --ignore-src -r -y && . /opt/ros/${ROS_DISTRO}/setup.sh && catkin build

# Create entrypoint
SHELL ["/bin/bash", "-c"]
COPY ./entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash