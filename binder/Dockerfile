FROM multiverseframework/giskard:noetic

ARG ROS_DISTRO=noetic
ENV REPO_DIR=/home/jovyan/repo
USER root

# Create non-root user jovyan
ENV SHELL=/bin/bash
ARG NB_USER=jovyan
ARG NB_UID=1000
ENV USER ${NB_USER}
ENV NB_UID ${NB_UID}
ENV HOME /home/${NB_USER}
RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${NB_UID} \
    ${NB_USER}

# Install apt packages
RUN apt-get update && apt-get install -y curl vim xvfb htop ros-${ROS_DISTRO}-tf2-tools
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash - &&\
    apt-get install -y nodejs

# Allow user jovyan write some directories
WORKDIR /home/Multiverse/giskard_ws
RUN chown -R ${NB_UID} /home/Multiverse/multiverse_ws/logs && \
    chown -R ${NB_UID} /home/Multiverse/multiverse_ws/src && \
    chown -R ${NB_UID} /home/Multiverse/giskard_ws && \
    ln -s  /home/Multiverse/giskard_ws /home/jovyan/giskard_ws

RUN pip install pip --upgrade
# Install jupyterlab and related packages
RUN pip install --no-cache-dir notebook==6.4.12 jupyterlab==3.6.5 \
        pyOpenSSL==23.2.0 pyyaml==5.3.1 cryptography==38.0.4 \
        jupyter-resource-usage jupyter-offlinenotebook \
        jupyter-server-proxy jupyterlab-git jlab-enhanced-cell-toolbar \
        autobahn gnupg twisted cbor2 empy ipywidgets  \
        pymongo Pillow pycryptodomex simplejpeg \
        https://raw.githubusercontent.com/yxzhan/jupyterlab-rviz/master/release/jupyterlab_rviz-0.2.4.tar.gz


# Install Rvizweb and ROSBoard
USER ${NB_USER}
RUN git clone https://github.com/yxzhan/rvizweb.git src/rvizweb -b master
RUN git clone https://github.com/yxzhan/rosboard.git src/rosboard -b main
COPY --chown=${NB_USER}:users ./binder/app.json src/rvizweb/webapps/
RUN rosdep update
USER root
RUN rosdep install -y --ignore-src --from-paths ./ -r
RUN chown -R ${NB_UID} /home/jovyan
USER ${NB_USER}
RUN catkin build

COPY --chown=${NB_USER}:users . ${REPO_DIR}
COPY --chown=${NB_USER}:users ./binder/multiverse_params.yaml /home/Multiverse/multiverse_ws/src/multiverse_core/multiverse_server/config/
COPY --chown=${NB_USER}:users ./binder/entrypoint.sh /home/${NB_USER}/.local/

# Compress IAI apartment 3d model texture
RUN python3 ${REPO_DIR}/binder/compress_textures.py \
    --img_path=/home/Multiverse/multiverse_ws/src/multiverse_world/iai_apartment\
    --max_size=512

USER ${NB_USER}
WORKDIR ${HOME}
ENTRYPOINT ["/home/jovyan/.local/entrypoint.sh"]