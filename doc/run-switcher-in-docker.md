# `shmdata`, `switcher` and `scenic` in docker

In this page you will find instruction about:

* How to setup your host in order to run nvidia compatible docker images
* How to run scenic and switcher with access to nvdia GPU for the video window and GPU video encoding
* How to run scenic and switcher in a Raspi with docker
* How to build your own scenic, switcher and shmdata images

This document need to be updated for Ubuntu 20.04.
{: .alert .alert-gitlab-orange}


## Install the [nvidia docker runtime](https://developer.nvidia.com/nvidia-container-runtime) in ubuntu 18.04

The minimum nvidia drivers version on the host is **418**.

### 1. Ensure you have [a correct docker version](https://docs.docker.com/install/linux/docker-ce/ubuntu/)

```bash
sudo -S apt-get remove docker docker-engine docker.io

sudo apt-get install \
  apt-transport-https \
  ca-certificates \
  curl \
  software-properties-common

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
  sudo apt-key add -

sudo add-apt-repository \
  "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) \
  stable"

sudo apt update
sudo apt-get install docker-ce

# Finally, Add group docker to current user
sudo usermod -a -G docker $USER
```

### 2. List informations about your nvidia device, as seen from nvidia runtime

```bash
sudo nvidia-container-cli --load-kmods info
```

### 3. Install [the nvidia runtime for Ubuntu 18.04](https://github.com/NVIDIA/nvidia-docker#ubuntu-140416041804-debian-jessiestretch)

```bash
# If you have nvidia-docker 1.0 installed: we need to remove it and all existing GPU containers
docker volume ls -q -f driver=nvidia-docker | \
  xargs -r -I{} -n1 docker ps -q -a -f volume={} | \
  xargs -r docker rm -f

sudo apt-get purge -y nvidia-docker

# Add the package repositories
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  sudo apt-key add -

DISTRIBUTION=$(. /etc/os-release;echo $ID$VERSION_ID)

curl -s -L https://nvidia.github.io/nvidia-docker/${DISTRIBUTION}/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update

# Install nvidia-docker2 and reload the Docker daemon configuration
sudo apt-get install -y nvidia-docker2
sudo pkill -SIGHUP dockerd

# Test nvidia-smi with the latest official CUDA image
docker run --runtime=nvidia --rm nvidia/cuda:9.0-base nvidia-smi
```

## Run `switcher` from built images

A docker image of `switcher` is built into its [registry](https://gitlab.com/sat-metalab/switcher/container_registry). You can start building your own library on top of this image by pulling it.

Three tags of this image are provided :

| tag                | purpose     | description                                                            |
|--------------------|-------------|------------------------------------------------------------------------|
| [master][master]   | production  | Clean image based on the `master` branch and build with `Release` flag |
| [develop][develop] | development | Clean image based on the `develop` branch and build with `Debug` flag  |
| [ci][ci]           | testing     | Image used for CI and used for unit tests                              |

[master]: registry.gitlab.com/sat-metalab/switcher:master
[develop]: registry.gitlab.com/sat-metalab/switcher:develop
[ci]: registry.gitlab.com/sat-metalab/switcher:ci

### Run `switcher` from remote image with nvidia support

```bash
xhost +si:localuser:root

docker run \
  -p 8080:8080 \
  -p 8000:8000 \
  --runtime=nvidia \
  --device /dev/snd \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -it registry.gitlab.com/sat-metalab/switcher:develop \ # or use tag develop
  bash
```

### Build and run `switcher` from you localy built image

```bash
xhost +si:localuser:root

# build switcher image from your local source...
docker build \
  --tag=switcher:develop \
  --build-arg SHMDATA_VERSION=develop
  -f dockerfiles/Dockerfile .

# ...and run it
docker run \
  -p 8080:8080 \
  -p 8000:8000 \
  --runtime=nvidia \
  --device /dev/snd \
  --device /dev/video0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -it switcher:develop
```

### Run switcher with access to sound

```bash
docker run \
  --device /dev/snd \
  -it registry.gitlab.com/sat-metalab/switcher:develop \
  /bin/bash
```

### Run switcher with access to a jack server running on the host

```bash
docker run \
  --volume=/dev/shm:/dev/shm:rw \
  --user=$(cat /etc/passwd | grep $(whoami) | cut -d":" -f3) \
  -it registry.gitlab.com/sat-metalab/switcher:develop \
  /bin/bash
```

### Build your own images

You need to build three images: [`shmdata`][shmdata_docker], `switcher` and [`scenic`][scenic_docker]. Each one depends on the previous.

[shmdata_docker]: https://gitlab.com/sat-metalab/shmdata/tree/develop#docker-images
[scenic_docker]: https://gitlab.com/sat-mtl/telepresence/scenic

The `switcher` image uses [mutli-stage builds][docker-multi-stage] with three stages : `dependencies`, `build` and `clean`. Theses stages use some build arguments :

| arguments       | stages              | description                              | default         |
|-----------------|---------------------|------------------------------------------|-----------------|
| SHMDATA_IMAGE   | `dependencies`      | The `shmdata` base image                 | `registry.gitlab.com/sat-metalab/shmdata`
| SHMDATA_TAG     | `dependencies`      | The **tag** of the `shmdata` base image  | `develop`       |
| BUILD_DIR       | `build` and `clean` | Where `shmdata` source is copied         | `/opt/switcher` |
| BUILD_TYPE      | `build`             | The build type of `shmdata`              | `Release`       |
| ENABLE_GPL      | `build`             | Flag which enables SIP and video features| `ON`            |
| C_COMPILER      | `build`             | Chosen compiler for C language files     | `gcc-8`         |
| CXX_COMPILER    | `build`             | Chosen compiler for CXX language files   | `g++-8`         |

[docker-multi-stage]: https://docs.docker.com/develop/develop-images/multistage-build/

In addition, an environment variable is set for the Nvidia runtime :

| variable                   | description                                                      | default |
|----------------------------|------------------------------------------------------------------|---------|
| NVIDIA_DRIVER_CAPABILITIES | See [nvidia container runtime environment variables][nvidia-env] | `all`   |

[nvidia-env]: https://github.com/NVIDIA/nvidia-container-runtime#nvidia_driver_capabilities

All images can be built and tested from source :

```bash
# build switcher with the "build" stage, all unused dependencies are not removed
docker build -t switcher:test -f dockerfiles/Dockerfile --target build .

# execute bash into the BUILD_DIR folder
docker run -ti switcher:test bash

# execute switcher unit tests
cd build
make test
```

### Push your images to Docker Hub

Here follows the commands that push a local image called _shmdata_ into the _nbouillot_ account 

```bash
docker login # you will be prompted for your duckerhub login and password
docker tag image_id nbouillot/shmdata:latest
docker push nbouillot/shmdata
```

# Raspeberry pi 3

Install the Raspbian stretch operating system in your Raspi following instructions [here](https://www.raspberrypi.org/downloads/raspbian/). Then, install docker as follows:

```bash
curl -fsSL get.docker.com -o get-docker.sh && sh get-docker.sh
sudo usermod -a -G docker $USER
```

You can run `scenic` using remote images with [Docker Compose](https://docs.docker.com/compose/):

```bash
git clone https://gitlab.com/sat-mtl/telepresence/scenic.git && cd scenic
docker-compose up # ...and you'll be able to control scenic from localhost:8080
```

You can build you own images:

```bash
# Build the shmdata image
git clone https://gitlab.com/sat-metalab/shmdata.git && cd shmdata

docker build \
  --tag=shmdata:raspi \
  -f dockerfiles/raspi.Dockerfile \
  .

# Build the switcher image
git clone https://gitlab.com/sat-metalab/switcher.git && cd switcher

docker build \
  --tag=switcher:raspi \
  --build-arg SHMDATA_IMAGE=shmdata \
  --build-arg SHMDATA_TAG=raspi \
  -f dockerfiles/raspi.Dockerfile \
  .

# Build the scenic-core image
git clone https://gitlab.com/sat-mtl/telepresence/scenic-core.git && cd scenic-core

docker build \
  --tag=scenic:raspi \
  --build-arg SWITCHER_IMAGE=switcher \
  --build-arg SWITCHER_TAG=raspi \
  -f dockerfiles/raspi.Dockerfile \
  .
```

# Some links
-------

* [Write Dockerfiles](https://docs.docker.com/develop/develop-images/dockerfile_best-practices)
* [Deal with nvidia specifics](https://devblogs.nvidia.com/gpu-containers-runtime/)
