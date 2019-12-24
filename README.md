switcher
========

![Switcher logo](doc/Switcher_horizontal_shadow_C.png)


[![build status](https://gitlab.com/sat-metalab/switcher/badges/master/build.svg)](https://gitlab.com/sat-metalab/switcher/commits/master)

[switcher](https://gitlab.com/sat-metalab/switcher) is an integration environment, able to interoperate with other software and protocols. Switcher provides low latency streaming of multichannel audio, video and data through IP networks.

Switcher is more generic and provides managing of several instances of services (called Quiddities). A Quiddity can be created and removed dynamically and can be controlled through property get/set as well as method invocations and information tree monitoring. Switcher provides introspection mechanisms to help write higher level software written in C++, python3, node.js or shell. Switcher can save and load the state of quiddities. 

Most Quiddities expose and/or consume live streams of data frames using the [shmdata](https://gitlab.com/sat-metalab/shmdata), a library for sharing data streams among processes with zero copy through POSIX shared memory. Note shmdata has [GStreamer](https://gstreamer.freedesktop.org/) elements.

For instance, the camera (v4l2src) Quiddity provides the video stream from a camera that can be connected simultaneously to several other Quiddities, including a local display (glfwin), a file recorder (avrec), a low-latency streamer (SIP) or more, including another shmdata enabled software. Note that the SIP quiddity supports NAT traversal through STUN/TURN and a companion repository is available for deployment of a SIP server compatible with switcher: [scenic-server](https://gitlab.com/sat-metalab/scenic-server). 

[Scenic](https://gitlab.com/sat-metalab/scenic) provides a web interface for switcher. It emphasizes routing of audio, video and data signals and multichannel transmission over the network.

[NDI2shmdata](https://gitlab.com/sat-metalab/ndi2shmdata) converts [shmdata](https://gitlab.com/sat-metalab/shmdata) to [NewTek's NDI](http://ndi.newtek.com), and _vice versa_. Note that you can include shmdata from external applications using the _extshmsrc_ quiddity. 

For more details about the various Quiddities, see [here](doc/quiddity_types.txt).

See instructions about:
- [Switcher installation](doc/INSTALL.md)
- [Switcher and scenic in docker](doc/run-switcher-in-docker.md)
- [Python3 scripting](doc/python-scripting.md)
- [Switcher configuration file](doc/configuration.md)
- [Mapping OSC or HTTP messages to quiddity properties](doc/protocol-mapper.md)
- [Using the GStreamer plugin for NVidia H264 GPU decoder (nvdec)](doc/using-nvdec-gstreamer-plugins.md)
- [Using the Executor quiddity](plugins/executor/README.md)
- [Shell scripting switcher and scenic](doc/shell-scripting.md)
- [Shell scripting a SIP call](doc/sip-call.md)
- [Shell scripting OSC quiddities](doc/using-osc-quiddities.md)
- [Contributing code](doc/contributing.md)
- [Code structure](doc/code-structure.md)
- [Writing custom quiddity bundles](doc/writing-bundles.md)

Command line
-------
switcher can be started with the ```switcher``` command and can be controlled remotely with the ```switcher-ctrl``` command. Help can be obtained with the ```-h``` option.

License
-------
switcher is released under the terms of the GNU GPL v3 or above.
libswitcher is released under the terms of the GNU LGPL v2 or above.
switcher plugins licenses are per plugin, see plugin folders.

In addition, the developers of the switcher hereby grants permission for non-GPL compatible GStreamer plugins to be used and distributed together with switcher. This permission is above and beyond the permissions granted by the GPL license by which switcher is covered. If you modify this code, you may extend this exception to your version of the code, but you are not obligated to do so. If you do not wish to do so, delete this exception statement from your version.

Some optional parts of switcher are licensed under the GNU General Public License
version 2 or later (GPL v2+). None of these parts are used by default, you have to explicitly pass -DENABLE\_GPL=ON to cmake to activate them. In this case, libswitcher's license changes to GPL v2+.

Specifically, the GPL parts of libswitcher are:
switcher-pjsip
switcher-v4l2
switcher-gsoap
switcher-resample

Authors
-------
See [here](AUTHORS.md).

Sponsors
--------
Switcher was created at the Society for Arts and Technology (SAT) to give to artists a powerful tool for telepresence in contexts of live arts and new media installations. Scenic/Switcher is the streaming engine used in the [Scènes ouvertes](http://sat.qc.ca/en/scenes-ouvertes) project: a network of more than 20 venues collaborating through artistic telepresence installations in Quebec.

This project is made possible thanks to the Society for Arts and Technologies. [SAT](http://www.sat.qc.ca/) and to the Ministère de l'Économie, de la Science et de l'Innovation du Québec (MESI).

