FROM multiverseframework/multiverse-core:ubuntu-20.04

RUN rosdep init && rosdep fix-permissions && ./build_multiverse_ws.sh

# Create entrypoint
SHELL ["/bin/bash", "-c"]
COPY ./entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash