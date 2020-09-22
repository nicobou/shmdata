INSTALL   
=======

> **Note**: Ensure **[Shmdata](https://gitlab.com/sat-metalab/shmdata)** is already installed before proceeding.

## Quick build and installation (Ubuntu 20.04)

Build and install **switcher** from the command line:

```bash
# Install all dependencies
sudo apt install bison build-essential cmake flex freeglut3-dev gsoap gstreamer1.0-libav gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-tools jackd libcgsi-gsoap-dev libcurl4-gnutls-dev libgl1-mesa-dev libglib2.0-dev libglu1-mesa-dev libgstreamer-plugins-bad1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libjack-jackd2-dev libjson-glib-dev liblo-dev libltc-dev libportmidi-dev libpulse-dev libsamplerate0-dev libsoup2.4-dev libssl-dev libtool libvncserver-dev libxcursor-dev libxinerama-dev libxrandr-dev linux-libc-dev mesa-common-dev python3-dev python3-pip swh-plugins uuid-dev wah-plugins xorg-dev

pip3 install pyOpenSSL websockets

# Clone all code from master branch
git clone https://gitlab.com/sat-metalab/switcher.git

# Configure build folder
cd switcher
git submodule update --init --recursive
mkdir build && cd build

# Generate make recipes
cmake .. -DENABLE_GPL=ON -DCMAKE_BUILD_TYPE=Release # replace "Release" with "Debug" when coding

# Build and install switcher on your system
make -j"$(nproc)"
sudo make install && sudo ldconfig
```

## Quick build and installation (Ubuntu 18.04)

Build and install **switcher** from the command line:

```bash
# Install all dependencies
sudo apt install cmake bison build-essential flex libtool libglib2.0-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev libjson-glib-dev libcgsi-gsoap-dev gstreamer1.0-libav gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly liblo-dev linux-libc-dev libpulse-dev libportmidi-dev libjack-jackd2-dev jackd libvncserver-dev uuid-dev libssl-dev swh-plugins  libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev libltc-dev libcurl4-gnutls-dev gsoap wah-plugins libxrandr-dev libxinerama-dev libxcursor-dev libsamplerate0-dev  libsoup2.4-dev python3-dev gcc-8 g++-8 libxxf86vm-dev

# Clone all code from master branch
git clone https://gitlab.com/sat-metalab/switcher.git

# Configure build folder
cd switcher
git submodule update --init --recursive
mkdir build && cd build

# Generate make recipes
CC="gcc-8" CXX="g++-8" cmake .. -DENABLE_GPL=ON -DCMAKE_BUILD_TYPE=Release # replace "Release" with "Debug" when coding

# Build and install switcher on your system
make -j"$(nproc)"
sudo make install && sudo ldconfig
```

> The inline environment variables `CC` and `CXX` are set in order to force the usage of **gcc-8** and **g++-8** without polluting your system environment. All following instructions assume you are using **gcc-8** and **g++-8** as C/C++ compilers.

## Custom compilation

You can verify and change the build configuration using **ccmake**. To do so, you must first install the needed package:
    
```
$ sudo apt install cmake-curses-gui
```

Then, after running `$ cmake ..`, from the build directory run:

```
$ ccmake ..
```
    
It will display a list of the configuration variables for the build.

When running non-interactive cmake you have to set the ENABLE\_GPL option if you want SIP and video features, otherwise they will be disabled by default:
```
$ cmake .. -DENABLE_GPL=ON
```

## Build Nvidia Video Encoding plugin

1. Check that you are running Nvidia drivers:

```
    $ nvidia-settings
```

2. Install Nvidia drivers (min. version `396`) and CUDA toolkit:

    > **Note**: You may need to first add the PPA `graphics-drivers/ppa`:  
    > `$ sudo add-apt-repository --yes ppa:graphics-drivers/ppa`

```
    $ sudo apt install nvidia-driver-<driver-ver-number> nvidia-dkms-<driver-ver-number> nvidia-kernel-common-<driver-ver-number> nvidia-kernel-source-<driver-ver-number>
    $ sudo apt install nvidia-cuda-toolkit nvidia-cuda-dev  # CUDA
```

3. In case running `$ cmake ..` does not automatically detect the right driver, in the **switcher** build directory, configure **switcher** as follows:

```
    $ cmake .. -DNVIDIA_PATH=/usr/lib/nvidia-<driver-ver-number>
```

    For example, replacing `<driver-ver-number>` with the installed Nvidia driver version:

```
    $ cmake .. -DNVIDIA_PATH=/usr/lib/nvidia-396
```

4. Lastly, compile and install as usual:

```
    $ make -j"$(nproc)"
    $ sudo make install
```

If you wish to use GPU-accelerated video decoding, you will also need to build the Gstreamer nvdec plugin. Instructions for doing so are [here](doc/using-nvdec-gstreamer-plugins.md).

## Other build options

* To run the tests:

```
    $ make test
```

* To generate installation packages (as configured in `CMakeLists.txt`):

```
    $ make package
```

* To generate a source package:

```
    $ make package_source
```

* To test the source package, this will create the source package and then try to build and test it

```
    $ make package_source_test
```

# Mac OS X Installation (experimental)

* Install xcode

Then install command line tools. From the terminal:
```
xcode-select --install
```

* Install **[Homebrew](https://github.com/Homebrew/brew)**:

```
    $ /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

* Install dependencies:

```
    $ brew install cmake pkg-config gsoap glib json-glib gstreamer gst-libav gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly liblo portmidi python3 libltc curl ossp-uuid jack qjackctl libltc curl 
```

* Build and install:

```
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ sudo make install
```

## Notes

* If you are encountering `-lintl` errors when using **Homebrew** to install dependencies, run the following command:

```
    $ brew link gettext
```
