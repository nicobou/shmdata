INSTALL   
=======

Quick build and installation (latest Ubuntu)
--------------------------------------------
First, make sure you have [shmdata](https://github.com/sat-metalab/shmdata) installed. Then here is how to build and install switcher from command line:

~~~~~~~~~~~~~~~~~~~~~
    # mandatory dependencies
    sudo apt-get install automake bison build-essential flex libtool
    sudo apt-get install libglib2.0-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libjson-glib-dev libcgsi-gsoap-dev
    # dependencies for building plugins 
    sudo apt-get install liblo-dev  linux-libc-dev libgtk2.0-dev libpulse-dev libportmidi-dev libjack-jackd2-dev libvncserver-dev
    # gstreamer plugins
    sudo apt-get install gstreamer1.0-libav gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly
    ./autogen.sh
    ./configure
    make -j
    sudo make install
    sudo ldconfig
~~~~~~~~~~~~~~~~~~~~~

Build nvenc plugin
-------------------

1. check you are runing nvidia drivers:
~~~~~~~~~~~~~~~~~~~~~
    nvidia-settings
~~~~~~~~~~~~~~~~~~~~~
2. download and unzip nvenc SDK version 5.0.1:
https://developer.nvidia.com/nvidia-video-codec-sdk
3. install cuda with headers
download and install cuda 7.5 from here:
https://developer.nvidia.com/cuda-downloads
then install cuda headers
~~~~~~~~~~~~~~~~~~~~~
    sudo apt-get update
    sudo apt-get install cuda nvidia-cuda-dev
~~~~~~~~~~~~~~~~~~~~~
4. in the switcher directory, configure switcher as follow
~~~~~~~~~~~~~~~~~~~~~
    NVENC_SDK_PATH='<your-path>' NVENC_LIBS='-L/usr/lib/nvidia-<driver-version-number>' ./configure
~~~~~~~~~~~~~~~~~~~~~
replacing information by yours, for instance:
~~~~~~~~~~~~~~~~~~~~~
NVENC_SDK_PATH='/var/tmp/nvenc_5.0.1_sdk' NVENC_LIBS='-L/usr/lib/nvidia-352' ./configure
~~~~~~~~~~~~~~~~~~~~~
5. compile and install as usual
~~~~~~~~~~~~~~~~~~~~~
make
sudo make install
~~~~~~~~~~~~~~~~~~~~~
