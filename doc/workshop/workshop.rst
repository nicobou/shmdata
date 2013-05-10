SHMDATA
=======

About
-----
Library to share flows of data frames between processes via shared memory. 
shmdata can be compared to the JACK audio connection kit project or to the
VideoJack project. shmdata, however, does not provide a host server and require applications to link data streams using socket path (e.g. "/tmp/my_shmdata_stream"). 

Note that shmdata readers and writers can hot connect/disconnect/reconnect/... 
and that a shared memory supports a single writer with multiple readers.

Installation
------------
Here is how to build and install it on Debian GNU/Linux or Ubuntu
::

  $ sudo apt-get install automake bison build-essential flex libtool 
  $ sudo apt-get install libglib2.0-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
  $ sudo apt-get install puredata-dev libsamplerate0-dev
  $ ./autogen.sh && ./configure && make && make check && sudo make install && sudo ldconfig

Example with text
-----------------
Objective: exchanging text between two applications (write and read text to a shmdata)
::
  
  $ cd examples/
  $ ./data_writer /tmp/myshmdatatext

in an other terminal::

  $ cd examples/
  $ ./data_reader /tmp/myshmdatatext

one writer can feed multiple readers, open an other terminal::

  $ cd examples/
  $ ./data_reader /tmp/myshmdatatext

ctrl-C the writer and relaunch, the reader dynamically reattach to it. 

shmdata are compatible with gstreamer
-----------------------------------------
Objective: use with GStreamer and gst-launch
::

  $ cd examples/
  $ ./data_writer /tmp/myshmdatatext

in an other terminal
::

  $ gst-launch videotestsrc ! "video/x-raw-yuv, width=1024, height=800" ! textoverlay name=to ! xvimagesink shmsrc socket-path=/tmp/truc ! gdpdepay ! to.

In this example the reader is not dynamically reconnecting because the gstreamer element shmsrc is not capable to reconnect.

shmdata writer, pulseaudio and puredata
---------------------------------------
Objective: getting audio from puredata into a GStreamer pipeline using pulseaudio
::

  $ rm /tmp/pd_testshm ; padsp pd -oss 1.pd
  $ gst-launch -v shmsrc is-live=true socket-path=/tmp/pd_testshm ! gdpdepay ! decodebin2 ! pulsesink sync=false

shmdata writer, jack and puredata
---------------------------------
Objective: same as previous but playing playing through jack (ok, maybe not the best use of shmdata here ;)

Known issue: pd blocksize must be equal or higher than jack blocksize
::
 
  $ /usr/bin/jackd -r -dalsa -dhw:1,0 -r44100 -p64 -n2 -Xseq
  $ rm /tmp/pd_testshm ; pd -jack -blocksize 64 shmdata-1.pd
  $ gst-launch -v shmsrc is-live=true socket-path=/tmp/pd_testshm ! gdpdepay ! decodebin2 ! jackaudiosink sync=false


shmdata writer and reader
-------------------------
Objective: reading audio shmdata into pd
::

  $ rm /tmp/pd_testshm ; padsp pd  -blocksize 512 -oss shmdata-2.pd

writing code for a writer (libshmdata-any)
------------------------------------------
Objective: look to a sample code about how to write into shmdata, basically, give a description string and push buffers
::

  $ sudo make install
  $ cp examples/data-writer.c ~/tmp/
  $ cd ~/tmp
  $ gcc data-writer.c -o datawriter `pkg-config --cflags --libs shmdata-any-0.8`

writing code for a reader (libshmdata-any)
------------------------------------------
Objective: same as previous but for reading
::

  $ sudo make install
  $ cp examples/data-reader.c ~/tmp/
  $ cd ~/tmp
  $ gcc data-reader.c -o datareader `pkg-config --cflags --libs shmdata-any-0.8`


SWITCHER
========

Switcher is designed for being a framework enabling distributed communication of media streams between remote processes with extensive use of the publish/subscribe paradigm.
The use of switcher is based on creating modules (called quiddities) that have properties and methods. Dynamic control of module is done through network protocols of library calls (libswitcher). Currently available network protocols are Open Sound Control (OSC) and Simple Object Access Protocol (SOAP).

Of interest here in this workshop, many quiddities are "segment" of GStreamer pipeline communicating through shmdata. In order to connect two segments, the reading quiddity (sink) can be invoked with the "connect" method which take the shmdata path as argument... more to come in the examples.   

Note 1: The word quiddity actually means "whatever makes something the type that it is" (Merriam Webster). This is accordingly an appropriate name for "stuff" or "something" or an abstract cpp class. 
Note 2: quiddities will have signal in the next 2 weeks
 
install
-------
Objective: install from sources

Note: you need OSC PureData externals. Following examples uses OSCtx and OSCrx
::

  $ sudo apt-get install automake bison build-essential flex libtool
  $ sudo apt-get install libglib2.0-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libjson-glib-dev liblo-dev
  $ ./autogen.sh && ./configure && make && sudo make install && sudo ldconfig


help
----
Objective: explore the use of the command line switcher and switcher-ctrl

"switcher" is the server that can be dynamically controlled with the "switcher-ctrl" tool. "switcher-ctrl" allow for listing instantiated quiddities, invoking quiddity methods and get/set quiddity properties. Notice that both tools provide options introspecting properties and methods of quiddities.
::

  $ switcher -h
  $ switcher-ctrl -h

create quiddity, invoke and remove quiddity
-------------------------------------------
Objective: first creating of a quiddity from puredata

Note: shmdata path are of form </tmp/switcher_default_quiddityname_suffix, 
where "default" is the name of the switcher server, "quiddityname" is the name you gave at create and suffix is determined by the quiddity itself
::

  $ switcher -d --osc-port 7770
  $ rm /tmp/pd_testshm ; padsp pd -oss switcher-1.pd

register to switcher properties (get the logs)
----------------------------------------------
Objective: get notifid of internal changes (subscribe to a property and switcher logs) 

switcher logger is a quiddity with a string property call "last-line"
::

  $ switcher -d --osc-port 7770
  $ rm /tmp/pd_testshm ; padsp pd -oss switcher-2.pd

play Big Buck Bunny from the web 
--------------------------------
Objective: play a movie from the web with the audio into puredata
::

  $ switcher -d --osc-port 7770
  $ /usr/bin/jackd -r -dalsa -dhw:1,0 -r44100 -p64 -n2 -Xseq
  $ rm /tmp/pd_testshm ; pd -jack switcher-3.pd

stream to a location
--------------------
Objective: stream and receive audio to localhost
::

  $ switcher -d --osc-port 7770
  $ /usr/bin/jackd -r -dalsa -dhw:1,0 -r44100 -p64 -n2 -Xseq
  $ rm /tmp/pd_testshm ; pd -jack switcher-4.pd

This last example shows that you can stream shmdatas to a location, receive it and re-stream it somewhere. You can therefore build a complex communication graph of live data streaming, selecting which stream you send at a specific location. This has been illustrated with the waterfall music concert last Thursday night ! 
