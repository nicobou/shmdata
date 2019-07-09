             __            __     __      
        ___ / /  __ _  ___/ /__ _/ /____ _
       (_-</ _ \/  ' \/ _  / _ `/ __/ _ `/
      /___/_//_/_/_/_/\_,_/\_,_/\__/\_,_/ 

[![build status](https://gitlab.com/sat-metalab/shmdata/badges/master/build.svg)](https://gitlab.com/sat-metalab/shmdata/commits/master)

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
        
# Docker installation

A docker image of `shmdata` is built into its [registry](https://gitlab.com/sat-metalab/shmdata/container_registry). You can start building your own library on top of this image by pulling it. Two versions of this image are provided : one based on the `develop` branch (unstable) and another based on the `master` branch (stable).

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



# Mac OS installation
* Install homebrew

        /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"

* Install dependencies

        brew install cmake pkg-config gstreamer gst-plugins-base python3

* Build & Install

        mkdir build
        cd build
        cmake ..
        make
        sudo make install


## Mac OS Notes
* If you are using homebrew to install dependencies and encountering errors about ```-lintl```, you have to ```brew link gettext```
