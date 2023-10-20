FROM multiverseframework/multiverse-core:ubuntu-20.04

# Copy packages
COPY Multiverse/multiverse_ws /home/Multiverse/multiverse_ws
COPY Multiverse/build_multiverse_ws.sh /home/Multiverse/build_multiverse_ws.sh

RUN rosdep init && rosdep fix-permissions && ./build_multiverse_ws.sh

# Create entrypoint
SHELL ["/bin/bash", "-c"]
COPY ./entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash