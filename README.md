# Multiverse-Docker

This repository provides an example of how to execute Multiverse Framework within a Docker environment.

## Prerequisites

Ensure that you have [Docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) installed.

Verify your versions of `docker` and `docker compose` by running:

```bash
$ docker -v
Docker version 27.1.2, build d01f264
$ docker compose version
Docker Compose version v2.29.1
```

### 1. Set up Docker

Run the following commands to configure Docker:

```bash
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
