             __            __     __      
        ___ / /  __ _  ___/ /__ _/ /____ _
       (_-</ _ \/  ' \/ _  / _ `/ __/ _ `/
      /___/_//_/_/_/_/\_,_/\_,_/\__/\_,_/ 

[![build status](https://gitlab.com/sat-metalab/shmdata/badges/master/build.svg)](https://gitlab.com/sat-metalab/shmdata/commits/master)

# shmdata
Library to share streams of framed data between processes via shared memory. shmdata is server less: it requires applications to link data streams using socket path (e.g. "/tmp/my_shmdata_stream"). Shmdata is very fast and allows process to access data streams without the need of extra copy.

The communication paradigm is 1 to many, i.e. one writer is making available data frames to several followers. Followers and writers are able to hot connect & disconnect. A shmdata can be resized during transmission and shmdata are typed using a string description published by writer at each reconnections. The description format is a user defined string.

Some examples :

* [C++](tests/check-writer-follower.cpp)
* [C](test/check-c-wrapper.cpp)
* [Python3](wrappers/python/example.py)
* [GStreamer writer](gst/check-shmdatasink.c)
* [GStreamer follower](gst/check-shmdatasrc.c)

# Use compiled GStreamer plugins with GStreamer tools:

By default, gstreamer plugins are installed in ```/usr/local/lib/gstreamer-1.0/```.

```
gst-inspect-1.0 --gst-plugin-path=/usr/local/lib/gstreamer-1.0/ shmdatasrc
gst-inspect-1.0 --gst-plugin-path=/usr/local/lib/gstreamer-1.0/ shmdatasink
```

# Installation
Here is how to build and install it on Debian GNU/Linux or Ubuntu::

    $ sudo apt install cmake bison build-essential flex libtool
    $ # this is only for building gstreamer plugins:
    $ sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
    $ # this is for building python wrappers 
    $ sudo apt-get install python3-dev
    $ mkdir build
    $ cd build
    $ cmake ..
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
        
* To test the source package, this will create the source package and then try to build and test it

        make package_source_test
        

# Mac OS Installation
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
