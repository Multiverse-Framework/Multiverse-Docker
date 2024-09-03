FROM multiverseframework/multiverse-tutorials:ubuntu-20.04@sha256:679fb4741ef2bf375fe02c4070a09c2f4d84e1c313a501dd653997722a22973a

USER root
ENV SHELL=/bin/bash

# Create non-root user jovyan
ENV NB_USER=jovyan
ENV NB_UID=1000
ENV USER ${NB_USER}
ENV NB_UID ${NB_UID}
ENV HOME /home/${NB_USER}
# RUN adduser --disabled-password \
#     --gecos "Default user" \
#     --uid ${NB_UID} \
#     ${NB_USER}

# Install apt packages
RUN apt-get update && apt-get install -y curl vim xvfb htop ros-noetic-tf2-tools
# Install nodejs (dependency of Jupyterlab extensions)
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash - &&\
    apt-get install -y nodejs

# Install jupyterlab and related python packages
RUN pip install pip --upgrade
RUN pip install --no-cache-dir --ignore-installed notebook==6.4.12 jupyterlab==3.6.5 \
    wstool pyOpenSSL==23.2.0 pyyaml==5.3.1 cryptography==38.0.4 \
    jupyter-resource-usage jupyter-offlinenotebook \
    jupyter-server-proxy jupyterlab-git jlab-enhanced-cell-toolbar \
    autobahn gnupg twisted cbor2 empy ipywidgets  \
    pymongo Pillow pycryptodomex simplejpeg tqdm \
    https://raw.githubusercontent.com/yxzhan/jupyterlab-rviz/master/dist/jupyterlab_rviz-0.3.1.tar.gz \
    empy==3.3.4

# Allow user ${NB_USER} write some directories
RUN chown -R ${NB_UID} /tmp
RUN chown -R ${NB_UID} /home/Multiverse/multiverse/modules/multiverse_knowledge/Konclude
RUN chown -R ${NB_UID} /home/Multiverse/multiverse/modules/multiverse_connectors/scripts/utils/config
RUN chown -R ${NB_UID} /home/Multiverse/multiverse/resources
RUN chown -R ${NB_UID} /home/Multiverse/multiverse/saved
RUN chown -R ${NB_UID} /home/Multiverse/multiverse_ws
RUN chown -R ${NB_UID} /home/Multiverse/multiverse_ws2
RUN chown -R ${NB_UID} /home/Multiverse-Tutorials/resources
RUN mkdir /home/Multiverse-Tutorials/output && chown -R ${NB_UID} /home/Multiverse-Tutorials/output
RUN chown -R ${NB_UID} /home/Multiverse-Tutorials/tutorials/multiverse_client

ARG MULTIVERSE_DIR=/home/Multiverse/multiverse
ARG BUILD_DIR=$MULTIVERSE_DIR/build
RUN /bin/bash -c "source /usr/local/bin/virtualenvwrapper.sh; mkvirtualenv --system-site-packages multiverse"
RUN /bin/bash -c "source /usr/local/bin/virtualenvwrapper.sh; workon multiverse; pip install pyside6 pyopengl pyside6 wheel cython owlready2 markupsafe==2.0.1 jinja2 pybind11 urdf_parser_py mujoco==3.2.0 scipy panel jupyter-server bash_kernel; python -m bash_kernel.install"
RUN /bin/bash -c "source /usr/local/bin/virtualenvwrapper.sh; workon multiverse; cmake -S $MULTIVERSE_DIR -B $BUILD_DIR -DCMAKE_INSTALL_PREFIX:PATH=$MULTIVERSE_DIR -DMULTIVERSE_CLIENT_LIBRARY_TYPE=STATIC -DSTDLIB=libstdc++"
RUN /bin/bash -c "source /usr/local/bin/virtualenvwrapper.sh; workon multiverse; make -C $BUILD_DIR"
RUN /bin/bash -c "source /usr/local/bin/virtualenvwrapper.sh; workon multiverse; cmake --install $BUILD_DIR"

# Copy necessary files
COPY --chown=${NB_USER}:users binder /home/binder

USER root
RUN apt update && rosdep install -y --ignore-src --from-paths /home/Multiverse/multiverse_ws/src -r && rosdep install -y --ignore-src --from-paths /home/Multiverse/multiverse_ws2/src -r
RUN chown -R ${NB_UID} /home/${NB_USER} && \
    chown ${NB_UID} /home/Multiverse
USER ${NB_USER}

# Install OH-My-Bash
RUN bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh)" --unattended && \
    sed -i 's/OSH_THEME="font"/OSH_THEME="half-life"/' /home/${NB_USER}/.bashrc
COPY --chown=${NB_USER}:users binder/jupyter-settings.json /usr/local/share/jupyter/lab/settings/overrides.json

RUN echo "export PYTHONPATH=$PYTHONPATH:$MULTIVERSE_DIR/lib/dist-packages" >> ~/.bashrc
RUN echo "source /usr/local/bin/virtualenvwrapper.sh && workon multiverse" >> ~/.bashrc

USER ${NB_USER}
WORKDIR /home/Multiverse-Tutorials
ENTRYPOINT ["/home/binder/entrypoint.sh"]
