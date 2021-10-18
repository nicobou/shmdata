GStreamer Elements
==================

Two GStreamer elements are provided:
* `shmdatasink`: create a shmdata writer from the end of a GStreamer pipeline
* `shmdatasrc`: create a Shmdata follower that becomes a source in the GStreamer pipeline

An example of use is provided in the [Shmdata repository README](../../README.md) file.

Documentation about the these element can be obtained using the `gst-inpect-1.0` command:
```
gst-inspect-1.0 shmdatasink
gst-inspect-1.0 shmdatasrc
```

