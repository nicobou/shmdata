NEWS
====
Here you will find a high level list of new features and bugfixes for each releases. 

shmdata 1.3.50 (2021-04-19)
---------------------------
This is an official release in the 1.3 stable series.

* ✨ Added framerate and frame duration counters

shmdata 1.3.48 (2021-04-07)
---------------------------
This is an official release in the 1.3 stable series.

* 🐛 Fix nullptr segfault in copy_to_shm

shmdata 1.3.46 (2021-03-22)
---------------------------
This is an official release in the 1.3 stable series.

* 📝 Improved contributing file and added Code of Conduct
* 📝 Added Shmdata principle diagram
* 📝 Updated getting started

shmdata 1.3.44 (2021-03-08)
---------------------------
This is an official release in the 1.3 stable series.

* 📝 fixed display video from shmdata in README

shmdata 1.3.42 (2021-02-22)
---------------------------
This is an official release in the 1.3 stable series.

* Fixed packaging script not keeping updated changelog
* 🎨 Reordered text and improved readme

shmdata 1.3.40 (2021-02-08)
---------------------------
This is an official release in the 1.3 stable series.

* 🐛 Fixed Ubuntu packaging script

shmdata 1.3.38 (2020-12-14)
---------------------------
This is an official release in the 1.3 stable series.

* 🐛 Fixed packaging issue where patches were not updated correctly when merging the upstream

shmdata 1.3.36 (2020-11-30)
---------------------------
This is an official release in the 1.3 stable series.

* Added an Ubuntu packaging script

shmdata 1.3.34 (2020-11-16)
---------------------------
This is an official release in the 1.3 stable series.

Packaging and CI:
* 👷 add Debian package build in CI with Ubuntu 20.04
* 📦 generate Debian package with make package
* 🐛 Fixed running tests without having to install

Improvement:
* 🔥 remove IntelliJ IDEA files

shmdata 1.3.32 (2020-10-20)
---------------------------
This is an official release in the 1.3 stable series.

* ✅ update package_source_test, removed the need for shmdata being installed
* 👷 remove docker-in-docker from CI

shmdata 1.3.30 (2020-09-21)
---------------------------
This is an official release in the 1.3 stable series.

* 🍱 adding logo and graphic charter
* ✨ Add extra-caps-properties to GstShmdataSink element
* 🐛 fix compilation with gcc 10

shmdata 1.3.28 (2020-07-14)
---------------------------
This is an official release in the 1.3 stable series.

* 🐛 Fix segfault in copy_to_shm
* 🐛 Fix segfault on server connection

shmdata 1.3.26 (2020-06-01)
---------------------------
This is an official release in the 1.3 stable series.

* 📝 adding documentation about sdflow in README.md

shmdata 1.3.24 (2020-04-20)
---------------------------
This is an official release in the 1.3 stable series.

* 💬 a name for Marie-Eve

shmdata 1.3.22 (2020-01-31)
---------------------------
This is an official release in the 1.3 stable series.

* 🎨 Added a prefix to the logger's macros to avoid collision
* 📈 Added analytics for coverage in CI
* ✅ Added sdcrash test
* ✅ Added sdflow test

shmdata 1.3.20 (2019-09-16)
---------------------------
This is an official release in the 1.3 stable series.

* fix make package_source_test
* Unify shmdata docker dockerfiles and push images from CI
* improved int type parsing in caps
* fix compilation issue with ubuntu 19.04
* mention of NDi2shmdata in README.md
* pyshmdata installs in python module path instead of cmake prefix
* fix a sometime deadlock in pyshmdata
* pyshmdata example as test
* reformating code of python wrapper
* adding custom type names in type parser

shmdata 1.3.18 (2018-10-19)
---------------------------
This is an official release in the 1.3 stable series.

* add a type parser with its test
* fix install folder not the a same as mentionned in pkg-config

shmdata 1.3.16 (2018-09-19)
---------------------------
This is an official release in the 1.3 stable series.

* fix pyshmdata sometimes crash during on_connect
* fix pyshmdata leaking user data when deallocating
* fix pyshmdata issue with PyDict_SetItemString
* Added a parsed_datatype member to pyshmdata Reader, a dict of the parsed datatype

shmdata 1.3.14 (2018-04-23)
---------------------------
This is an official release in the 1.3 stable series.

* ubuntu 18.04 gitlab ci
* more informations in README.md
* LICENSE file filled
* AUTHORS.md file created

shmdata 1.3.12 (2018-04-06)
---------------------------
This is an official release in the 1.3 stable series.

* Changes plugin name from "shm" to "shmdata" to match the lib name because this is needed in gstreamer 1.14.
* cast self to GstAllocator in g_object_ref
* Use POSIX format for size_t printing.
* Added a short pyshmdata example
* Updated Python wrapper documentation
* Added call the PyEvalInitThreads to fix GIL segfault. Returns an error if trying to init a reader on something not a socket.

shmdata 1.3.10 (2016-12-12)
---------------------------
This is an official release in the 1.3 stable series.

* removing unnecessary lock in gstshmdatasink.c
* Added gitlab CI

shmdata 1.3.8 (2016-11-25)
---------------------------
This is an official release in the 1.3 stable series.

* fix from shmdata address if first buffer triggers a resize,

shmdata 1.3.6 (2016-11-11)
---------------------------
This is an official release in the 1.3 stable series.

* build fixes

shmdata 1.3.4 (2016-10-21)
---------------------------
This is an official release in the 1.3 stable series.

* Increase timeout during reader creation (100ms could be short when loading heavy save files). Fixed synchronization issue on socket file descriptor durin destruction.

shmdata 1.3.2 (2016-10-03)
---------------------------
This is an official release in the 1.3 stable series.

* fix minor release

shmdata 1.3.0 (2016-09-30)
---------------------------
This is an official release in the 1.3 stable series.

* Add a notification when connection to shmdata server is lost or acquired.

shmdata 1.2.10 (2016-09-12)
---------------------------
This is an official release in the 1.2 stable series.

* adding buffers stat in gst elements.

shmdata 1.2.8 (2016-08-04)
---------------------------
This is an official release in the 1.2 stable series.

* fix writer taking 100% CPU usage when reader already following shmdata

shmdata 1.2.6 (2016-07-20)
---------------------------
This is an official release in the 1.2 stable series.

* Delete the writer before recreating it if it already exists in sink_on_caps method.
* Fixed member order to avoid race condition in destruction.
* Lock was done too late, provoking a race condition when resizing shmdata.
* improved gstshmdatasink test
* Fixed pyshmdata compilation,
* removing hard coded python version
* use python3-config instead of pythpon3.4-config

shmdata 1.2.4 (2016-06-07)
---------------------------
This is an official release in the 1.2 stable series.

* improved gstshmdatasink test
* Fixed pyshmdata compilation,
* use python3-config instead of pythpon3.4-config

shmdata 1.2.2 (2016-04-04)
---------------------------
New features:

* none

Bugs fixed:

* various typos

shmdata 1.2.0 (2016-03-17)
---------------------------
New features:

* shmdata are resizing automatically
* getting shmmax and shmmni from system

Bugs fixed:

* various fixes in GStreamer elements

shmdata 1.0.2 (2015-08-17)
---------------------------
This is an official release in the 1.0 stable series. 

New features:

* There are no new features, this is complete rewrite of 0.8

Bugs fixed:

* none

shmdata 0.8.14 (2014-08-04)
---------------------------
This is an official release in the 0.8 stable series. 

New features:

* There are no new features, this is a bugfix release

Bugs fixed:

* avoid accumulating data in the writer (leaking)

shmdata 0.8.12 (2014-02-27)
---------------------------
This is an official release in the 0.8 stable series. 

New features:

* There are no new features, this is a bugfix release

Bugs fixed:

* removing deprecated init function
* fix osg library

shmdata 0.8.10 (2014-02-24)
---------------------------
This is an official release in the 0.8 stable series. 

New features:

* There are no new features, this is a bugfix release

Bugs fixed:

* NULL checking
* base reader and writter are thread safe
* clean signal handlers

shmdata 0.8.8 (2014-01-16)
---------------------------
This is an official release in the 0.8 stable series. 

New features:

* There are no new features, this is a bugfix release

Bugs fixed:

* Make check on OSX

shmdata 0.8.6 (2013-10-16)
---------------------------
This is the second official release in the 0.8 stable series. 

New features:

* There are no new features, this is a bugfix release

Bugs fixed:

* shmsrc~ cleaning properly resampled data
* Leaks in base-reader
* Leak affecting each buffer on base reader
* Error on Pd DSP on/off
* GLitching shmsrc~

shmdata 0.8.3 (2012-06-19)
---------------------------
This is the second official release in the 0.8 stable series. 

New features:

* base reader supports custom GMainContext
* gst elements are installed on the gstreamer sdk folder on osx

Bugs fixed:

* removed double unref when file exists

shmdata 0.8.0 (2012-04-25)
---------------------------
This is the first official release in the 0.8 stable series. It is identical to the 0.6.6 release. 

New features:

* see 0.6.6

Bugs fixed:

* see 0.6.6

shmdata 0.6.6 (2012-04-25)
---------------------------
This is an official release in the 0.6 stable series.

New features:

* shmsrc~ and shmsink~ for multi-channel audio with puredata
* more unit tests
* option to run g_main_loop internally or not (shmdata-any)
* option to use absolute timestamp or not
* port to osx
* adding caps discovery for user

Bugs fixed:

* gstreamer state management improvement
* fix bug when file exist and also detected by gfilemonitor

shmdata 0.6.4 (2012-10-03)
---------------------------
This is an official release in the 0.6 stable series.

Bugs fixed:

* Fix OSG tearing with HD video
* Add video-writer-uri example
* Fix freeze when plug is called from uridecodebin on_pad_added handler


shmdata 0.6.2 (2012-08-30)
---------------------------
This is an official release in the 0.6 stable series.

Bugs fixed:

* implementing a new interface for base reader, allowing to install or not the sync_handler
* fixing bus sync handler singleton issue
* calling close in video reader example


shmdata 0.6.0 (2012-07-18)
---------------------------
This is an official release in the 0.6 stable series.

Bugs fixed:

* occasional seg fault when deleting OSG reader
* occasionnal seg fault when deleting ba
