![Shmdata logo](doc/logo/png/Shmdata-color-horizontal-black-text.png)

[![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0) [![pipeline status](https://gitlab.com/sat-metalab/shmdata/badges/develop/pipeline.svg)](https://gitlab.com/sat-metalab/shmdata/commits/develop) [![coverage report](https://gitlab.com/sat-metalab/shmdata/badges/develop/coverage.svg)](https://gitlab.com/sat-metalab/shmdata/commits/develop)


# shmdata
Library to share streams of framed data between processes via shared memory. shmdata is server less: it requires applications to link data streams using socket path (e.g. "/tmp/my_shmdata_stream"). Shmdata is very fast and allows processes to access data streams without the need of extra copy.

The communication paradigm is 1 to many, i.e., one writer is making available data frames to several followers. Followers and writers can hot connect & disconnect. Shmdata transmission supports buffer resizing. Each shmdata has a type specified with a string description, itself published by the shmdata writer at each reconnection. The type is specified as a user-defined string. Although left to the user, we encourage type specification to follow GStreamer 1.0 caps specification format, for instance "audio/x-raw,format=S16LE,channels=2,layout=interleaved". 

Note the existence of [NDI2shmdata](https://gitlab.com/sat-metalab/ndi2shmdata) that converts shmdata to [NewTek's NDI](http://ndi.newtek.com), and _vice versa_.

Some examples :

* [C++](tests/check-writer-follower.cpp)
* [C](test/check-c-wrapper.cpp)
* [Python3](wrappers/python/example.py)
* [GStreamer writer](gst/check-shmdatasink.c)
* [GStreamer follower](gst/check-shmdatasrc.c)

# Use compiled GStreamer plugins with GStreamer tools:

By default, GStreamer plugins are installed in ```/usr/local/lib/gstreamer-1.0/```.

```
gst-inspect-1.0 --gst-plugin-path=/usr/local/lib/gstreamer-1.0/ shmdatasrc
gst-inspect-1.0 --gst-plugin-path=/usr/local/lib/gstreamer-1.0/ shmdatasink
```

# Create a video shmdata with GStreamer
The path of the shmdata will be /tmp/video_shmdata
```
gst-launch --gst-plugin-path=/usr/local/lib/gstreamer-1.0/ videotestsrc ! shmdatasink socket-path=/tmp/video_shmdata
```

# Monitor a shmdata
The `sdflow` utility is installed along with the shmdata library. It prints the shmdata metadata once connected with the shmdata writer, and then a line of information for each buffer pushed by the shmdata writer. Here is an example :
```
$ sdflow /tmp/video_shmdata 
connected: type video/x-raw, format=(string)I420, width=(int)320, height=(int)240, framerate=(fraction)30/1, multiview-mode=(string)mono, pixel-aspect-ratio=(fraction)1/1, interlace-mode=(string)progressive
0    size: 115200    data: EBEBEBEBEBEBEBEBEBEBEBEBEBEBEB...
1    size: 115200    data: EBEBEBEBEBEBEBEBEBEBEBEBEBEBEB...
etc
```

# Monitor frame rate of a shmdata

You need the `pv` utily:
```
sudo apt install pv
```

With `pv` and `sdflow` you can display the frame rate of a shmdata (here `/tmp/video_shmdata`):
```
sdflow /tmp/video_shmdata | pv --line-mode --rate > /dev/null
```

You can get this as a command (`sdfps`) if you copy the following into your `~/.bashrc` file:
```
function sdfps {
        sdflow $1 | pv --line-mode --rate > /dev/null
}
```

Then you can monitor the rate of a shmdata like this:
```
source ~/.bashrc # this reloads the bashrc configuration
sdfps /tmp/video_shmdata
```

# Installation
Here is how to build and install it on Debian GNU/Linux or Ubuntu::

    $ sudo apt install cmake bison build-essential flex libtool git
    $ # this is only for building gstreamer plugins:
    $ sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
    $ # this is for building python wrappers 
    $ sudo apt install python3-dev
    $ git clone https://gitlab.com/sat-metalab/shmdata.git
    $ cd shmdata
    $ mkdir build
    $ cd build
    $ cmake -DCMAKE_BUILD_TYPE=Release .. # replace "Release" with "Debug" when coding
    $ make
    $ sudo make install
    $ sudo ldconfig
  
  
## Other build options

* To run the tests

        make test
    
* To generate installation packages (as configured in CMakeLists.txt)

        make package
        
* To generate a source package

        make package_source
        
* To test the source package, this will create the source package, then try to build and test it

        make package_source_test
        
# Docker images

A docker image of `shmdata` is built into its [registry](https://gitlab.com/sat-metalab/shmdata/container_registry). You can start building your own library on top of this image by pulling it.

Three tags of this image are provided :

| tag                | purpose     | description                                                            |
|--------------------|-------------|------------------------------------------------------------------------|
| [master][master]   | production  | Clean image based on the `master` branch and build with `Release` flag |
| [develop][develop] | development | Clean image based on the `develop` branch and build with `Debug` flag  |
| [ci][ci]           | testing     | Image used for CI and used for unit tests                              |

[master]: registry.gitlab.com/sat-metalab/shmdata:master
[develop]: registry.gitlab.com/sat-metalab/shmdata:develop
[ci]: registry.gitlab.com/sat-metalab/shmdata:ci

## Install with Docker

1. Install Docker ([instructions for Ubuntu 18.04](https://docs.docker.com/install/linux/docker-ce/ubuntu/))

2. Pull the Docker image

```bash
docker pull registry.gitlab.com/sat-metalab/shmdata:develop # or use "master" tag
```

3. Configure Nvidia runtime (from [instructions](https://gitlab.com/sat-metalab/switcher/blob/develop/doc/run-switcher-in-docker.md#install-the-nvidia-docker-runtime-in-ubuntu-1804))

4. Develop your own tools from `shmdata`
    
    * You can take inspiration from [the Switcher and Scenic build instructions](https://gitlab.com/sat-metalab/switcher/blob/develop/doc/run-switcher-in-docker.md#run-scenic-from-remote-image-with-nvidia-support)
    
    * You can create your own Dockerfile

    ```bash
    FROM registry.gitlab.com/sat-metalab/shmdata
    # your Dockerfile
    ```

## Contribute with Docker

The `shmdata` image uses [mutli-stage builds][docker-multi-stage] with three stages : `dependencies`, `build` and `clean`. Theses stages use some build arguments :

| variables  | stages              | description                      | default        |
|------------|---------------------|----------------------------------|----------------|
| BUILD_DIR  | `build` and `clean` | Where `shmdata` source is copied | `/opt/shmdata` |
| BUILD_TYPE | `build`             | The build type of `shmdata`      | `Release`      |

All images can be built and tested from source :

```bash
# build shmdata with the "build" stage, all unused dependencies are not removed
docker build -t shmdata:test -f dockerfiles/Dockerfile --target build .

# execute bash into the BUILD_DIR folder
docker run -ti shmdata:test bash
```

[docker-multi-stage]: https://docs.docker.com/develop/develop-images/multistage-build/

# Mac OS installation

**Note**: we had success building this image on OSX with Homebrew, but it is not supported by the shmdata contributors, and it is not included in our continuous integration pipeline: it might be broken when you read this.

1. Install homebrew

```bash
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

2. Install dependencies

```bash
brew install cmake pkg-config gstreamer gst-plugins-base python3
```

3. Build & Install
```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

## Mac OS Notes
* If you are using homebrew to install dependencies and encountering errors about ```-lintl```, you have to ```brew link gettext```
