INSTALL   
=======

## Quick build and installation (latest Ubuntu)

> **Note**:  
> Ensure **[shmdata](https://github.com/sat-metalab/shmdata)** is already installed before proceeding.

Build and install **switcher** from the command line:

```
$ sudo apt-get install automake bison build-essential flex libtool libglib2.0-devlibgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libjson-glib-dev libcgsi-gsoap-devgstreamer1.0-libav gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly liblo-dev linux-libc-dev libgtk-3-dev libpulse-dev libportmidi-dev libjack-jackd2-dev libvncserver-dev
$ ./autogen.sh
$ ./configure
$ make -j"$(nproc)"
$ sudo make install
$ sudo ldconfig
```

## Build Nvidia NVENC plugin


1. Check that you are running Nvidia drivers:

    ```
    $ nvidia-settings
    ```

1. Install Nvidia drivers and CUDA toolkit

    > **Note**:  
    > You may need to first add the PPA `graphics-drivers/ppa`:  
    > `$ sudo add-apt-repository --yes ppa:graphics-drivers/ppa`

    ```
    $ sudo apt-get install nvidia-<driver-ver-number> nvidia-<driver-ver-number>-dev nvidia-cuda-toolkit
    ```

1. In the **switcher** directory, configure **switcher** as follows:

    ```
    $ NVENC_LIBS='-L/usr/lib/nvidia-<driver-ver-number>' ./configure
    ```

    For example, replacing `<driver-ver-number>` with the installed Nvidia driver version:

    ```
    $ NVENC_LIBS='-L/usr/lib/nvidia-367' ./configure
    ```

1. Compile and install as usual.
    
    ```
    $ make -j"$(nproc)"
    $ sudo make install
    ```
