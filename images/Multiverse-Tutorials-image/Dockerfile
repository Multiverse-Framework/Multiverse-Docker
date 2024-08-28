FROM multiverseframework/multiverse:ubuntu-20.04

RUN /home/jovyan/.virtualenvs/multiverse/bin/python3.8 -m pip install prior
RUN cd /home; git clone https://github.com/Multiverse-Framework/Multiverse-Tutorials.git
RUN cd /home/Multiverse-Tutorials/src; git clone https://github.com/allenai/ai2thor.git --depth 1
RUN cd /home/Multiverse-Tutorials/src; python get_meshes.py; python3.11 extract_objects.py
WORKDIR /home/Multiverse-Tutorials