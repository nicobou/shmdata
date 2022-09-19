# Installation from sources
Follow these instructions to build and install the library on Debian GNU/Linux or Ubuntu::

    $ sudo apt install cmake build-essential git
    $ # to build GStreamer plugins:
    $ sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
    $ # to build python wrappers 
    $ sudo apt install python3-dev
    $ git clone https://gitlab.com/sat-mtl/tools/shmdata.git
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
    
* To generate a debian installation packages (as configured in CMakeLists.txt)

        make package
        
* To generate a source package (sources in a tgz file) 

        make package_source
        
* To test the source package, this will create the source package, then try to build and test it

        make package_source_test


## Mac OS installation

**Note**: we've had success building this image on OSX using Homebrew, but this is not supported by the shmdata contributors, and it is not tested in our continuous integration pipeline: it might be broken by the time you read this.

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
