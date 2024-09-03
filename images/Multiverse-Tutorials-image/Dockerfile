FROM multiverseframework/multiverse:ubuntu-20.04

RUN mkdir -p /home/Multiverse-Tutorials/src
RUN cd /home/Multiverse-Tutorials/src; git clone https://github.com/allenai/ai2thor.git --depth 1
RUN /home/jovyan/.virtualenvs/multiverse/bin/python3.8 -m ipykernel install --user --name multiverse --display-name "multiverse"
RUN /home/jovyan/.virtualenvs/multiverse/bin/python3.8 -m pip install -U /home/Multiverse/multiverse/build/modules/multiverse_parser/multiverse_parser-0.0.1.tar.gz
RUN /home/jovyan/.virtualenvs/multiverse/bin/python3.8 -m pip install prior ipywidgets solara inflection
RUN pip install ipywidgets solara widgetsnbextension
RUN jupyter nbextension enable --py widgetsnbextension

COPY Multiverse-Tutorials /home/Multiverse-Tutorials
RUN cd /home/Multiverse-Tutorials/src; python get_meshes.py; python3.11 extract_objects.py

WORKDIR /home/Multiverse-Tutorials