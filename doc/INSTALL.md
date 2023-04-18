INSTALL   
=======

> **Note**: Ensure **[Shmdata](https://gitlab.com/nicobou/shmdata)** is already installed before proceeding.

## Install package

```
sudo add-apt-repository ppa:sat-metalab/metalab
sudo apt-get update
sudo apt install switcher python3-switcher
```

You may want to install some of the following packages:

```
sudo apt install switcher-dev # using libswitcher or writing you own Quiddity
sudo apt install switcher-doc # documentation
sudo apt install switcher-plugins-nvidia # install the nvidia video encoding Quiddity
```

## Quick build and installation (Ubuntu 22.04)

Build and install **switcher** from the command line:

```bash
# clone repository
sudo apt install git
git clone https://gitlab.com/nicobou/switcher.git

cd switcher

# uncomment to checkout a specific version
# git checkout 2.1.38

# install dependencies
sudo apt install $(cat ./deps/apt-{build,runtime}-ubuntu-22.04)

# nvidia dependencies: uncomment next line to build video encoding with the nvidia graphics card
# sudo apt install $(cat ./deps/apt-{build,runtime}-nvidia-deps-ubuntu-22.04)

# python dependencies
pip3 install -r ./deps/pip3-ubuntu22.04

# update or initialize git submodules recursively
git submodule update --init --recursive

# Generate make recipes
cmake -B build -DENABLE_GPL=ON -DCMAKE_BUILD_TYPE=Release # replace "Release" by "Debug" for development

# compile and install program on the system
make -sC build -j`nproc`
sudo make install -sC build && sudo ldconfig
```

## Quick build and installation (Ubuntu 22.04)

Build and install **switcher** from the command line:

```bash
# clone repository
sudo apt install git
git clone https://gitlab.com/nicobou/switcher.git

cd switcher

# uncomment to checkout a specific version
# git checkout 2.1.38

# install dependencies
sudo apt install $(cat ./deps/apt-{build,runtime}-ubuntu-22.04)

# nvidia dependencies: uncomment next line to build video encoding with the nvidia graphics card
# sudo apt install $(cat ./deps/apt-{build,runtime}-nvidia-deps-ubuntu-22.04)

# update or initialize git submodules recursively
git submodule update --init --recursive

# Generate make recipes
cmake -B build -DENABLE_GPL=ON -DCMAKE_BUILD_TYPE=Release # replace "Release" by "Debug" for development

# compile and install program on the system
make -sC build -j`nproc`
sudo make install -sC build && sudo ldconfig
```

## Custom compilation

You can verify and change the build configuration using **ccmake**. To do so, you must first install the needed package:
    
```
$ sudo apt install cmake-curses-gui
```

Then, after running `cmake -B build`, from inside the build directory use:

```
$ ccmake ..
```
    
It will display a list of the configuration variables for the build.

When running non-interactive cmake you have to set the ENABLE\_GPL option if you want SIP and video features, otherwise they will be disabled by default:
```
$ cmake -B build -DENABLE_GPL=ON
```

## Build against a custom NVIDIA driver (for GPU video encoding)

During the build process, if `$ cmake ..` does not automatically detect the right driver, in the **switcher** build directory, configure **switcher** as follows:

```
    $ cmake -B build -DNVIDIA_PATH=/usr/lib/nvidia-<driver-ver-number>
```

    For example, replacing `<driver-ver-number>` with the installed Nvidia driver version (here 396):

```
    $ cmake -B build -DNVIDIA_PATH=/usr/lib/nvidia-396
```

Then, compile and install as usual:

```
    $ make -sC build -j`nproc`
    $ sudo make install -sC build
```

Note: if you wish to use GPU-accelerated video decoding, you will also need to build the GStreamer nvdec plugin. Instructions for doing so are [here](doc/using-nvdec-gstreamer-plugins.md).

## Other build options

* To run the tests:

```
    $ make test
```

* To generate debian installation packages (as configured in `CMakeLists.txt`):

```
    $ sudo apt install file
    $ make package # <- generate a debian package
```

* To generate a source package:

```
    $ make package_source
```

* To test the source package, this will create the source package and then try to build and test it

```
    $ make package_source_test
```

