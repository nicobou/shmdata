shmdata
=======

![Shmdata logo](doc/logo/png/Shmdata-color-horizontal-black-text.png)

[![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0) [![pipeline status](https://gitlab.com/sat-metalab/shmdata/badges/develop/pipeline.svg)](https://gitlab.com/sat-metalab/shmdata/commits/develop) [![coverage report](https://gitlab.com/sat-metalab/shmdata/badges/develop/coverage.svg)](https://gitlab.com/sat-metalab/shmdata/commits/develop)

Library to share streams of framed data between processes via shared memory. shmdata is server less: it requires applications to link data streams using socket path (e.g. "/tmp/my_shmdata_stream"). Shmdata is very fast and allows processes to access data streams without the need of extra copy.

The communication paradigm is 1 to many, i.e., one writer is making available data frames to several followers. Followers and writers can hot connect & disconnect. Shmdata transmission supports buffer resizing. Each shmdata has a type specified with a string description, itself published by the shmdata writer at each reconnection. The type is specified as a user-defined string. Although left to the user, we encourage type specification to follow GStreamer 1.0 caps specification format, for instance "audio/x-raw,format=S16LE,channels=2,layout=interleaved". 

Note the existence of [NDI2shmdata](https://gitlab.com/sat-metalab/ndi2shmdata) that converts shmdata to [NewTek's NDI](http://ndi.newtek.com), and _vice versa_.

Some examples :

* [C++](tests/check-writer-follower.cpp)
* [C](test/check-c-wrapper.cpp)
* [Python3](wrappers/python/example.py)
* [GStreamer writer](gst/check-shmdatasink.c)
* [GStreamer follower](gst/check-shmdatasrc.c)

## Use compiled GStreamer plugins with GStreamer tools:

By default, GStreamer plugins are installed in ```/usr/local/lib/gstreamer-1.0/```.

```
gst-inspect-1.0 --gst-plugin-path=/usr/local/lib/gstreamer-1.0/ shmdatasrc
gst-inspect-1.0 --gst-plugin-path=/usr/local/lib/gstreamer-1.0/ shmdatasink
```

## Create a video shmdata with GStreamer
The path of the shmdata will be /tmp/video_shmdata
```
gst-launch --gst-plugin-path=/usr/local/lib/gstreamer-1.0/ videotestsrc ! shmdatasink socket-path=/tmp/video_shmdata
```

## Monitor a shmdata
The `sdflow` utility is installed along with the shmdata library. It prints the shmdata metadata once connected with the shmdata writer, and then a line of information for each buffer pushed by the shmdata writer. Here is an example :
```
$ sdflow /tmp/video_shmdata 
connected: type video/x-raw, format=(string)I420, width=(int)320, height=(int)240, framerate=(fraction)30/1, multiview-mode=(string)mono, pixel-aspect-ratio=(fraction)1/1, interlace-mode=(string)progressive
0    size: 115200    data: EBEBEBEBEBEBEBEBEBEBEBEBEBEBEB...
1    size: 115200    data: EBEBEBEBEBEBEBEBEBEBEBEBEBEBEB...
etc
```

## Monitor frame rate of a shmdata

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

## Installation
Here is how to build and install it on Debian GNU/Linux or Ubuntu::

    $ sudo apt install cmake build-essential git
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
  
  
### Other build options

* To run the tests

        make test
    
* To generate installation packages (as configured in CMakeLists.txt)

        make package
        
* To generate a source package

        make package_source
        
* To test the source package, this will create the source package, then try to build and test it

        make package_source_test


## Mac OS installation

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

### Mac OS Notes
* If you are using homebrew to install dependencies and encountering errors about ```-lintl```, you have to ```brew link gettext```


## Contributions and advance uses

To contribute to shmdata, see the [contribution guide](doc/contributing.md)
