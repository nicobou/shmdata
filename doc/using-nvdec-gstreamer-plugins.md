# Using the nvdec GStreamer plugin

_Install and build instructions for Ubuntu 18.04._

Nvidia's nvenc and nvdec plugins enable GPU-accelerated H264 video encoding and decoding with GStreamer pipelines. Currently, GPU encoding is already included in Switcher, but decoding is not. To enable GPU decoding with the GStreamer nvdec plugin, you will need to i) build the nvdec plugin from source and ii) raise its prioritization as primary among the h264 decoders.

## Requirements
### Graphics card
A compatible Nvidia GPU must be installed on your system. A list of compatible devices is available here: https://developer.nvidia.com/video-encode-decode-gpu-support-matrix

### Drivers
The latest Nvidia drivers (currently version 415) must be installed:

```bash
sudo apt install nvidia-driver-415
```

### CUDA
Nvdec requires CUDA to be installed. Currently, with GStreamer 1.14, only version 9 is compatible with nvdec.

```bash
sudo apt install nvidia-cuda-toolkit
```

You can validate the installation by running `nvcc --version`.

### Nvidia Video Codec SDK
Building the plugins requires the user to install the Nvidia Video SDK. (version 8.2) The SDK is available here: https://developer.nvidia.com/nvidia-video-codec-sdk. It is free of charge, although it does require you to create a Nvidia account.

Unzip the downloaded archive and copy the necessary header files to `/usr/local/include`. The SDK is made of only 3 files; the rest are examples of CUDA applications.

```bash
cd Video_Codec_SDK_8.2.16/
# Encoder
sudo cp Samples/NvCodec/NvEncoder/nvEncodeAPI.h /usr/local/include/
# Decoder
sudo cp Samples/NvCodec/NvDecoder/cuviddec.h /usr/local/include/
sudo cp Samples/NvCodec/NvDecoder/nvcuvid.h /usr/local/include/
```

## Building and installing the GStreamer plugins
Starting with GStreamer 1.14, nvdec and nvenc are part of the `gst-plugins-bad` collection. These are installed by default with Ubuntu 18.04, except for nvdec and nvenc. They must be built manually

Start by cloning the `gst-plugins-bad` Git repository on your machine. This will clone the latest version of the plugins; however, it order to build them, `gst-plugins-bad` must be the exact same version as the installed GStreamer version, which is 1.14.1 on a default Ubuntu 18.04 install. We must checkout the 1.14.1 version of the `gst-plugins-bad` repo.

```bash
git clone https://gitlab.freedesktop.org/gstreamer/gst-plugins-bad.git
cd gst-plugins-bad
git checkout tags/1.14.1
```

Execute the following commands to build the plugins (in `gst-plugins-bad/`)

```bash
./autogen.sh --disable-gtk-doc --with-cuda-prefix=/usr
make
sudo make install
sudo ldconfig
```

_If the `--disable-gtk-doc` flag is omitted, you will need to install the `gtk-doc-tools` package before building the plugins._

The nvdec and nvenc plugins will be installed in `/usr/local/lib/gstreamer-1.0` as `libgstnvenc.so` and `libgstnvdec.so`.

Finally, in order for GStreamer to detect the 2 plugins, we must update the `GST_PLUGIN_PATH` environment variable:

```bash
GST_PLUGIN_PATH=$GST_PLUGIN_PATH:/usr/local/lib/gstreamer-1.0
```

If everything went smoothly, you should see both plugins (__nvdec__ and __nvh264enc__) in the list of detected plugins when running `gst-inspect-1.0`.

## Using nvdec as the default H264 decoder in Switcher
Switcher uses a GStreamer [__decodebin__](https://gstreamer.freedesktop.org/documentation/application-development/highlevel/playback-components.html) element in order to decode video streams. Decodebins are handy because they automatically identify, create and connect all the necessary plugins for decoding the supplied stream. 

However, by default, a decodebin will always use a CPU decoder when decoding H264 streams. In order to use the nvdec plugin instead, we must assign it a higher priority (or rank) than the CPU decoder. Decodebins always uses the decoder with the highest rank.

This can be easily done in the `switcher.json` file, by adding the following entry at the root of the configuration:

```json
"gstreamer" : {
    "primary_priority" : {
      "nvdec": 10
    }
 }
```

Inside `primary _priority`, simply specify a GStreamer plugin (in our case, __nvdec__), and assign it a priority (integer only). On Switcher's startup, any specified plugin will be assigned the `PRIMARY` rank + the specified number in `switcher.json`. In our case, nvdec will have a rank of `PRIMARY`+10, which is higher than the other decoders (they often have a rank of `PRIMARY` only).

More info on the inner workings of this logic is available [here](https://gstreamer.freedesktop.org/documentation/tutorials/playback/hardware-accelerated-video-decoding.html).
