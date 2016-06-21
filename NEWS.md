NEWS
====
Here you will find a high level list of new features and bugfixes for each releases. 

switcher 0.8.10 (2016-06-08)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* GTK geometry saving
* Added per-quiddity config
* Per-quiddity user tree
* Posture: Added a compress option to Scan3DGPU
* Add autoconnect feature to jack source and output.
* Reformatted code base with clang-format.
* untested syphonsrc port to shmdata1
* Posture: added PostureColorizeGL

Bug fixes:
* do not call contact if already calling it 
* Release script, usable by shmdata and switcher.
* fix nvenc 2Kx2K transmission with SIP
* adding a more complete test for nvenc plugin
* fix h264 in gst codec
* sip hang up is hanging up outgoing call or incoming call, but not both in the same invocation
* pjsip do not assert if cannot answer to call
* Fixed SIP connections graft/prune and empty array serialization for the tree
* Fixed an issue where only the last shmdata reader was updated in the info tree.
* Destroy "logger" quiddity last to keep proper formatting until the end.
* fix systemusage period property
* Fixed deadlocks and crashes when reloading SIP scenarii.
* Switch from hybrid sync/async messaging system in gst-pipeliner to using gstreamer watch mechanism.
* errors are fatal and crash the app (SIGTRAP), changed to a warning with error in the message.
* fix nvenc max number of simultaneous encode sessions reached
* Fixed a crash on disconnection of a source during a SIP call and an accidental creation of a reader key.
* fix video converter crash for large video,
* fix new call from already calling contact,
* fix load history file including create returning null
* Precommit hook now proposes to do the formatting automatically on the changes. 
* Added PostureMeshSerialization quiddity
* Update video caps on-the-fly when selecting them in Scenic.
* fix switcher-ctrl, renaming get_factory_capabilities in get_classes,
* disabling soap set_port after first invocation
* v4l2 shmdata writer is initialized with a larger value in order to avoid automatic resize for the first frame.
* v4l2src uses video-encoded suffix when the device produces non raw video
* GCC 5.3 is more restrictive than 5.1 and is used in Ubuntu 16.04. Soap needed an additional warning exception to build properly.
* fix fullscreen button latency with gtkwin
* fix gtk win properties when no video is connected
* fix gtk window shortcuts
* fix jack-to-shmdata error when deleted
* fix issue in shmdata-to-jack
* fix issue when property not found with soap
* fix missing arg issue with switcher-ctrl
* make timelapse multi shmreader
* num files and dir and nome parameters for timelapse
* max files in timelapse
* timelapse params can be changed live
* width and height in timelapse
* timelapse notifies last image in a read only property
* fix default video encoding is ugly on ubuntu 15.10
* fix seg when trying to register with network and without STUN/TURN
* fix v4l2 trying to compress video when camera is producing compressed video
* fix SIP cannot send to multiple destinations
* fix SIP call issue when no media add to SDP
* fix gtk property issues
* fix nvenc bug when start/stop input video
* fix lost property values for video-codec when start stop
* fix v4l2src capture configuration lost after start/stop
* fix gtk dtor bug
* fix some timelapse internals
* fix distcheck with posture
* PostureScan3DGPU: added preliminary support for mesh refinement
* PostureScan3DGPU: updated some limit values
* Posture: now checking in PostureSource and PostureScan3DGPU that the calibration file is alright
* more changes trying to fix posture transmission
* (pjsip) ports only determined by ICE
* removing several FIXME in pj-sip plugins
* removing some FIXMEs
* some simplification of pjsip plugin code
* fix Stack smashing detected when using stun with 4 shmdata
* fix race condition in rtpsender
* fix generated quid names collision after loading
* fix issue with saving/loading
* fix SIP shmdata byte_rate monitoring
* fix sip compilation with g++ 5.2
* fix missing string with g_warnings
* fix conflict with posture plugin
* fix duplicate byte rate info for shmdata writer in httpsdpdec
* Improved posture plugin threading
* removing old sockfd fix for OSX
* fix pruning with clang
* activating syphon for being ported to shmdata1
* fix check startable
* fix race condition with GstShmdataSubscriber
* no wait for state when asking PLAYING or PAUSED state
* disabling GTK on OSX
* add missing include for errno
* fix clang issue with gchar vs. string
* removing curl dependency
* removed hard coded switcher version in switcher-nodejs

switcher 0.8.8 (2016-04-04)
---------------------------
This is an official release in the 0.8 stable series. 

New features:

* video timelapse for multiple video shmdata
* option to number files or not
* max number of file for timelapse
* width and height in timelapse
* timelapse notifies last file written

Bugs fixed:

* default video encoding ugly on ubuntu 15.10
* segfault when trying to register with network and without STUN/TURN
* v4l2 trying to compress video when camera is producing compressed video
* SIP cannot send to multiple destinations
* SIP call issue when no media add to SDP
* gtk property issues
* nvenc bug when start/stop input video
* lost property values for video-codec when start stop
* v4l2src capture configuration lost after start/stop
* gtk dtor bug

switcher 0.8.6 (2016-03-21)
---------------------------
This is an official release in the 0.8 stable series. 

New features:

* property system refactored
* posture GPU
* nvenc support
* STUN/TURN support
* video timelapse

Bugs fixed:

* too many to be listed here

switcher 0.8.4 (2015-08-18)
---------------------------
This is an official release in the 0.8 stable series. 

New features:

* none

Bugs fixed:

* remove dependency to curl (not working with launchpad ppa)

switcher 0.8.2 (2015-08-17)
---------------------------
This is an official release in the 0.8 stable series. 

New features:

* none

Bugs fixed:

* switcher plugin not loaded from nodejs addon

switcher 0.8.0 (2015-08-17)
---------------------------
This is an official release in the 0.8 stable series. 

New features:

* SIP
* VNC (compatible with gtkwin)
* jack (resampling + autoconnection)
* audio encoder
* more option with v4l2 capture
* ported to gstreamer 1.0
* ported to shmdata 1.0
* using internal information tree

Bugs fixed:

* too many to be listed here

switcher 0.6.2 (2014-08-04)
---------------------------
This is an official release in the 0.6 stable series. 

New features:

* none
 
Bugs fixed:

* nodejs not loading plugins

switcher 0.6.0 (2014-08-04)
---------------------------
This is an official release in the 0.6 stable series. 

New features:

* information tree
* syphon (OSX)
* improved per-quiddity shmdata dynamic description
* shmdata-any support
* system usage quiddity
* SIP
* posture plugins
 
Bugs fixed:

* too many for being listed here 

switcher 0.4.6 (2014-03-07)
---------------------------
This is an official release in the 0.4 stable series. 

New features:

* none

Bugs fixed:

* node-switcher is better integrated with nodejs

switcher 0.4.4 (2014-03-05)
---------------------------
This is an official release in the 0.4 stable series. 

New features:

* adding node-switchon addons in sources

Bugs fixed:

* fix segfault in soap-ctrl-client
* make node-switcher encoding strings in utf8 instead of ascii

switcher 0.4.2 (2014-02-27)
---------------------------
This is an official release in the 0.4 stable series. 

New features:

* title property for gtkvideosink
* adding videoflip, video balance and gamma to gtk video sink

Bugs fixed:

* too many for being listed here

switcher 0.4.0 (2014-01-28)
---------------------------
This is an official release in the 0.4 stable series. 

New features:

* many video encoders are available with video-sources 
* dynamic presence of properties
* property mapper
* supporting plugins
* gtk video sink with fullscreen
* portmidi plugins
* pulse plugins
* v4l2 plugins
* dictionnary
* osc controler
* several internal features for developing quiddities
* no need to invoke set_runtime anymore

Bugs fixed:

* too many for being listed here

switcher 0.2.2 (2013-06-20)
---------------------------
This is an official release in the 0.2 stable series. 

New features:

* none

Bugs fixed:

* gsoap packaging

switcher 0.2.0 (2013-06-19)
---------------------------

This is the first official release in the 0.2 stable series. 

New features:

* signals
* save/load command history
* soap client

Bugs fixed:

* too many to listed here

switcher 0.1.2 (2013-04-26)
---------------------------
This is the first developer snapshot of switcher.

