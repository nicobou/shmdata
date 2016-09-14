INSTALL   
=======

Quick build and installation (latest Ubuntu)
--------------------------------------------
First, make sure you have [shmdata](https://github.com/sat-metalab/shmdata) installed. Then here is how to build and install switcher from command line:

~~~~~~~~~~~~~~~~~~~~~
    sudo apt-get install automake bison build-essential flex libtool libglib2.0-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libjson-glib-dev libcgsi-gsoap-dev gstreamer1.0-libav gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly liblo-dev  linux-libc-dev libgtk-3-dev libpulse-dev libportmidi-dev libjack-jackd2-dev libvncserver-dev
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
2. install nvidia drivers and CUDA toolkit
~~~~~~~~~~~~~~~~~~~~~
    sudo apt-get install nvidia-<driver-version-number> nvidia-cuda-toolkit
~~~~~~~~~~~~~~~~~~~~~
3. in the switcher directory, configure switcher as follow
~~~~~~~~~~~~~~~~~~~~~
   NVENC_LIBS='-L/usr/lib/nvidia-<driver-version-number>' ./configure
~~~~~~~~~~~~~~~~~~~~~
replacing information by yours, for instance:
~~~~~~~~~~~~~~~~~~~~~
NVENC_LIBS='-L/usr/lib/nvidia-361' ./configure
~~~~~~~~~~~~~~~~~~~~~
4. compile and install as usual
~~~~~~~~~~~~~~~~~~~~~
make
sudo make install
~~~~~~~~~~~~~~~~~~~~~
