FROM multiverseframework/giskard:ubuntu-20.04

RUN git remote set-url origin https://github.com/Multiverse-Framework/Multiverse.git && git checkout main && git pull origin main
RUN cd /home/Multiverse/multiverse_ws/src/multiverse_world && git remote set-url origin https://github.com/Multiverse-Framework/Multiverse-World.git && git checkout main && git pull origin main
RUN cd /home/Multiverse/multiverse_ws/src/multiverse_robots && git remote set-url origin https://github.com/Multiverse-Framework/Multiverse-Robots.git && git checkout main && git pull origin main
RUN cd /home/Multiverse/multiverse_ws/src/multiverse_objects && git remote set-url origin https://github.com/Multiverse-Framework/Multiverse-Objects.git && git checkout main && git pull origin main
RUN cd /home/Multiverse/multiverse_ws/src/multiverse_demos && git remote set-url origin https://github.com/Multiverse-Framework/Multiverse-Demos.git && git checkout main && git pull origin main
RUN cd /home/Multiverse/giskard_ws/src/giskardpy && git remote set-url origin https://github.com/Multiverse-Framework/giskardpy.git && git checkout main && git pull origin main

USER root
ENV SHELL=/bin/bash

# Create non-root user jovyan
ENV NB_USER=jovyan
ENV NB_UID=1000
ENV USER ${NB_USER}
ENV NB_UID ${NB_UID}
ENV HOME /home/${NB_USER}
RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${NB_UID} \
    ${NB_USER}

# Install apt packages
RUN apt-get update && apt-get install -y curl vim xvfb htop ros-noetic-tf2-tools
# Install nodejs (dependency of Jupyterlab extensions)
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash - &&\
    apt-get install -y nodejs

# Install jupyterlab and related python packages
RUN pip install pip --upgrade
RUN pip install --no-cache-dir notebook==6.4.12 jupyterlab==3.6.5 \
    wstool pyOpenSSL==23.2.0 pyyaml==5.3.1 cryptography==38.0.4 \
    jupyter-resource-usage jupyter-offlinenotebook \
    jupyter-server-proxy jupyterlab-git jlab-enhanced-cell-toolbar \
    autobahn gnupg twisted cbor2 empy ipywidgets  \
    pymongo Pillow pycryptodomex simplejpeg tqdm \
    https://raw.githubusercontent.com/yxzhan/jupyterlab-rviz/master/release/jupyterlab_rviz-0.2.5.tar.gz

# Allow user ${NB_USER} write some directories
WORKDIR /home/Multiverse/giskard_ws
RUN chown -R ${NB_UID} /tmp
RUN chown -R ${NB_UID} /home/Multiverse/multiverse/modules/multiverse_knowledge/Konclude
RUN chown -R ${NB_UID} /home/Multiverse/multiverse_ws
RUN chown -R ${NB_UID} /home/Multiverse/giskard_ws

# Copy necessary files
COPY --chown=${NB_USER}:users binder /home/binder
# COPY --chown=${NB_USER}:users binder/iros.jupyterlab-workspace /home/Multiverse/
COPY --chown=${NB_USER}:users binder/demo.sh /home/Multiverse/
# Fix the mesh obj url in marker array
COPY --chown=${NB_USER}:users binder/me /home/Multiverse/multiverse_ws/src/me

# Update every workspace
WORKDIR /home/Multiverse
RUN git config --global --add safe.directory /home/Multiverse && \
    git remote set-url origin https://github.com/Multiverse-Framework/Multiverse.git && \
    git pull origin main

# Install ROS web tools under workspace multiverse_ws
USER ${NB_USER}
WORKDIR /home/Multiverse/multiverse_ws/src
RUN wstool init && \
    wstool merge https://raw.githubusercontent.com/yxzhan/rvizweb/master/.rosinstall && \
    wstool update
RUN rosdep update
USER root
RUN apt update && rosdep install -y --ignore-src --from-paths ./ -r
RUN chown -R ${NB_UID} /home/${NB_USER} && \
    chown  ${NB_UID} /home/Multiverse
USER ${NB_USER}
RUN catkin build

# Install OH-My-Bash
RUN bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh)" --unattended && \
    sed -i 's/OSH_THEME="font"/OSH_THEME="half-life"/' /home/${NB_USER}/.bashrc
COPY --chown=${NB_USER}:users binder/jupyter-settings.json /usr/local/share/jupyter/lab/settings/overrides.json

# Instal bash kernel
RUN pip3 install panel jupyter-server bash_kernel
RUN python3 -m bash_kernel.install

USER ${NB_USER}
WORKDIR /home/Multiverse
ENTRYPOINT ["/home/binder/entrypoint.sh"]
