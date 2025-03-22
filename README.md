# Multiverse-Docker

This repository provides an example of how to execute Multiverse Framework within a Docker environment.

You can operate the Multiverse Framework with Docker in two ways: on the cloud or locally.

## Run Multiverse Docker on the Cloud using BinderHub

- [Link to the BinderHub](https://binder.intel4coro.de/v2/gh/Multiverse-Framework/Multiverse-Docker/ERF-2025-Ubuntu20.04)

For the IROS2025 experiments, you can quickly access these Jupyter Notebooks:

- [Universal Scene Description Parser Experiment (for URDF, MJCF and USDA as input)](https://binder.intel4coro.de/v2/gh/Multiverse-Framework/Multiverse-Docker/ICRA-2025?urlpath=lab%2Ftree%2FMultiverse-Tutorials%2Ftutorials%2Fmultiverse_parser_quick_start.ipynb)
- [Universal Scene Description Parser Experiment (for ProcTHOR as input)](https://binder.intel4coro.de/v2/gh/Multiverse-Framework/Multiverse-Docker/ICRA-2025?urlpath=lab%2Ftree%2FMultiverse-Tutorials%2Ftutorials%2Fmultiverse_knowledge_quick_start.ipynb)
- [Competency Questions Experiment](https://binder.intel4coro.de/v2/gh/sasjonge/semantic-map-lab.git/dfl_reasoner?labpath=notebooks%2Fsemantic_map.ipynb)

## Run Multiverse Docker locally

### Prerequisites

Ensure that you have [Docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) installed.

Verify your versions of `docker` and `docker compose` by running:

```bash
$ docker -v
Docker version 27.1.2, build d01f264
$ docker compose version
Docker Compose version v2.29.1
```

### 1. Set up Docker

Run the following commands to clone the repository and configure Docker:

```bash
git clone https://github.com/Multiverse-Framework/Multiverse-Docker.git && cd Multiverse-Docker
./setup_images.bash
docker compose pull
```

### 2. Start the Docker Compose service

To start the service, run the following command:

```bash
docker compose up
```

Once the service is running, open your browser and navigate to [http://127.0.0.1:8888/lab](http://127.0.0.1:8888/lab).

From there, you can either click on "Desktop" to access the virtual machine, or stay in the Jupyter notebook interface to explore and run tutorials.

The interface should appear similar to this:
![image](https://github.com/user-attachments/assets/f3a08557-0179-4dd9-a448-e878163e54ed)
