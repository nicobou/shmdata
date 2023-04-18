Switcher
========

![Switcher logo](doc/logo/png/Switcher-color-horizontal-black-text.png)


[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) [![pipeline status](https://gitlab.com/nicobou/switcher/badges/develop/pipeline.svg)](https://gitlab.com/nicobou/switcher/commits/develop) [![coverage report](https://gitlab.com/nicobou/switcher/badges/develop/coverage.svg)](https://gitlab.com/nicobou/switcher/commits/develop)

[Switcher](https://gitlab.com/nicobou/switcher) is an integration environment, able to interoperate with other software and protocols. Switcher provides low latency streaming of multichannel audio, video and data through IP networks.

Switcher provides managing of several instances of services (called Quiddities). A Quiddity can be created and removed dynamically and can be controlled through property get/set as well as method invocations and information tree monitoring. Switcher provides introspection mechanisms to help write higher level software written in python3 or C++. Switcher can save and load the state of quiddities.

Most Quiddities expose and/or consume live streams of data frames using the [shmdata](https://gitlab.com/nicobou/shmdata), a library for sharing data streams among processes with zero copy through POSIX shared memory. Note shmdata has [GStreamer](https://gstreamer.freedesktop.org/) elements.

For instance, the camera (v4l2src) Quiddity provides the video stream from a camera that can be connected simultaneously to several other Quiddities, including a local display (glfwin), a file recorder (avrec), a low-latency streamer (SIP) or more, including another shmdata enabled software. Note that the SIP quiddity supports NAT traversal through STUN/TURN and a companion repository is available for deployment of a SIP server compatible with Switcher: [sip-server](https://gitlab.com/nicobou/scenic/sip-server).

[Scenic](https://gitlab.com/nicobou/scenic) provides a web interface for Switcher. It emphasizes routing of audio, video and data signals and multichannel transmission over the network.

[NDI2shmdata](https://gitlab.com/nicobou/ndi2shmdata) converts [shmdata](https://gitlab.com/nicobou/shmdata) to [NewTek's NDI](http://ndi.newtek.com), and _vice versa_. Note that you can include shmdata from external applications using the _extshmsrc_ quiddity.

For more details about the various Quiddities, see [here](doc/quiddity_types.txt).

See instructions about the use of switcher:
- [Python3 scripting](doc/python-scripting.md)
- [Using the Executor quiddity](plugins/executor/README.md)
- [Mapping OSC or HTTP messages to quiddity properties](doc/protocol-mapper.md)
- [Using the GStreamer plugin for NVidia H264 GPU decoder (nvdec)](doc/using-nvdec-gstreamer-plugins.md)
- [Shell scripting Switcher and scenic](doc/shell-scripting.md)
- [Shell scripting a SIP call](doc/sip-call.md)
- [Shell scripting OSC quiddities](doc/using-osc-quiddities.md)
- [Using bundled tools](doc/tools.md)

About building switcher:
- [Switcher installation](doc/INSTALL.md)
- [Contributing code](doc/contributing.md)
- [Code structure](doc/code-structure.md)
- [Switcher and scenic in docker](doc/run-switcher-in-docker.md)

About switcher configuration:
- [Switcher configuration file](doc/configuration.md)
- [Writing custom quiddity bundles](doc/writing-bundles.md)
- [Loading quiddity plugins from custom path](doc/custom-quiddity-path.md)

License
-------
Switcher is released under the terms of the [GNU GPL v3 or above](LICENSE.md).
libswitcher is released under the terms of the GNU LGPL v2 or above.
Switcher plugins licenses are per plugin, see plugin folders.

In addition, the developers of the Switcher hereby grants permission for non-GPL compatible GStreamer plugins to be used and distributed together with Switcher. This permission is above and beyond the permissions granted by the GPL license by which Switcher is covered. If you modify this code, you may extend this exception to your version of the code, but you are not obligated to do so. If you do not wish to do so, delete this exception statement from your version.

Some optional parts of Switcher are licensed under the GNU General Public License
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
Switcher was created at the Society for Arts and Technology (SAT) to give to artists a powerful tool for telepresence in contexts of live arts and new media installations. Scenic/Switcher is the streaming engine used in the [Scènes ouvertes](http://sat.qc.ca/en/scenic-telepresence) project: a network of more than 20 venues collaborating through artistic telepresence installations in Quebec.

This project is made possible thanks to the Society for Arts and Technologies. [SAT](http://www.sat.qc.ca/) and to the Ministère de l'Économie, de l'Innovation et de l'Énergie du Québec (MÉIÉ).

