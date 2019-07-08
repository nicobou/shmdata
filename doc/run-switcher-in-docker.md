shmdata, switcher and scenic in docker
============

In this page you will find instruction about:

* how to setup your host in order to run nvidia compatible docker images
* how to run scenic and switcher with access to nvdia GPU for the video window and GPU video encoding
* how to run scenic and switcher in a Raspi with docker
* how to build your own scenic, switcher and shmdata images

Install the nvidia docker runtime in ubuntu 18.04 
-------

The minimum nvidia drivers version on the host is 418.

First you need to ensure you have the correct docker version
```
sudo -S apt-get remove docker docker-engine docker.io

sudo apt-get install \
     apt-transport-https \
     ca-certificates \
     curl \
     software-properties-common

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

sudo add-apt-repository \
     "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
    $(lsb_release -cs) \
    stable"

sudo apt update
sudo apt-get install docker-ce

# Finally, Add group docker to current user
sudo usermod -a -G docker $USER
```

List informations about your nvidi device, as seen from nvidia runtime:
```
sudo nvidia-container-cli --load-kmods info
```

Then install the runtime
```
# If you have nvidia-docker 1.0 installed: we need to remove it and all existing GPU containers
docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
sudo apt-get purge -y nvidia-docker

# Add the package repositories
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update

# Install nvidia-docker2 and reload the Docker daemon configuration
sudo apt-get install -y nvidia-docker2
sudo pkill -SIGHUP dockerd

# Test nvidia-smi with the latest official CUDA image
docker run --runtime=nvidia --rm nvidia/cuda:9.0-base nvidia-smi
```

Run switcher and Scenic from built images
-------

Run scenic from remote image with nvidia support
-------
```
xhost +si:localuser:root
docker run -p8080:8080 -p8000:8000 --runtime=nvidia --device /dev/snd -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY -e NVIDIA_VISIBLE_DEVICES=all -it nbouillot/scenic
```

Run scenic from you localy built image
```
xhost +si:localuser:root
docker run -p8080:8080 -p8000:8000 --runtime=nvidia --device /dev/snd --device /dev/video0 -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e NVIDIA_VISIBLE_DEVICES=all -it scenic:develop
```

Run switcher with access to sound
```
docker run --device /dev/snd -it switcher:develop /bin/bash
```

Run switcher with access to a jack server running on the host
```
docker run --volume=/dev/shm:/dev/shm:rw --user=$(cat /etc/passwd | grep $(whoami) | cut -d":" -f3) -it switcher:develop /bin/bash
```

Build your own images
------
You will build three images: shmdata, switcher and scenic. Each one depends on the previous.

```
cd ~
mkdir src
cd src
git clone https://gitlab.com/sat-metalab/switcher.git
cd switcher/docker
# building the shmdata image
docker build --tag=shmdata:develop --build-arg=BRANCH=develop shmdata/
docker build --tag=switcher:develop --build-arg BRANCH=develop --build-arg SHMDATA_BRANCH=develop switcher/
docker build --tag=scenic:develop --build-arg BRANCH=develop --build-arg SWITCHER_BRANCH=develop scenic/
```

Push your images to Docker Hub
----=
Here follows the commands that push a local image called _shmdata_ into the _nbouillot_ account 
```
docker login # you will be prompted for your duckerhub login and password
docker tag image_id nbouillot/shmdata:latest
docker push nbouillot/shmdata
```

Raspeberry pi 3
------
Install the Raspbian stretch operating system in your Raspi following instructions [here](https://www.raspberrypi.org/downloads/raspbian/). Then, install docker as follows:
```
curl -fsSL get.docker.com -o get-docker.sh && sh get-docker.sh
sudo usermod -a -G docker $USER
```

You can run scenic using remote images:
```

```

You can build you own image:
```
cd ~
mkdir src
cd src
git clone https://gitlab.com/sat-metalab/switcher.git
cd switcher/docker
# building the shmdata image
docker build --tag=shmdata:develop --build-arg=BRANCH=develop shmdata-raspi/
docker build --tag=switcher:develop --build-arg BRANCH=develop --build-arg SHMDATA_BRANCH=develop switcher-raspi/
docker build --tag=scenic:develop --build-arg BRANCH=develop --build-arg SWITCHER_BRANCH=develop scenic-raspi/
```

Some links
-------

* [Write Dockerfiles](https://docs.docker.com/develop/develop-images/dockerfile_best-practices)
* [Deal with nvidia specifics](https://devblogs.nvidia.com/gpu-containers-runtime/)
