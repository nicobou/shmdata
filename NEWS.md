NEWS
====
Here you will find a high level list of new features and bugfixes for each releases. 

shmdata 1.3.6 (2016-10-31)
---------------------------
This is an official release in the 1.3 stable series.

* Migration to CMake build system.

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
