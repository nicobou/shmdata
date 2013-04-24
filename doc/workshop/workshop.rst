shmdata
=======

About
-----

Library to share flows of data frames between processes via shared memory. 
shmdata can be compared to the JACK audio connection kit project or to the
VideoJack project. shmdata, however, does not provide a host server and require applications to link data streams using socket path (e.g. "/tmp/my_shmdata_stream"). 
Note that shmdata readers and writers can hot connect/disconnect/reconnect/... 
and that a shared memory supports a single writer with multiple readers.

License: LGPL for the libraries, GLP for the puredata external


Installation
------------
Here is how to build and install it on Debian GNU/Linux or Ubuntu::

  $ sudo apt-get install automake bison build-essential flex libtool 
  $ sudo apt-get install libglib2.0-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
  $ sudo apt-get install puredata-dev libsamplerate0-dev
  $ ./autogen.sh && ./configure && make && make check && sudo make install && sudo ldconfig


Example with text
-----------------

write and read text a shmdata (/tmp/myshmdatatext)
  $ cd examples/
  $ ./data_writer /tmp/myshmdatatext

#in an other terminal
  $ cd examples/
  $ ./data_reader /tmp/myshmdatatext

one writer can feed multiple readers, open an other terminal:
  $ cd examples/
  $ ./data_reader /tmp/myshmdatatext

ctrl-C the writer and relaunch, the reader dynamically reattach to it. 

shmdata are compatible with gstreamer
-----------------------------------------
  $ cd examples/
  $ ./data_writer /tmp/myshmdatatext

in an other terminal:
  $ gst-launch videotestsrc ! "video/x-raw-yuv, width=1024, height=800" ! textoverlay name=to ! xvimagesink shmsrc socket-path=/tmp/truc ! gdpdepay ! to.

here the reader is not dynamically reconnecting because the gstreamer element shmsrc is not capable to reconnect.


shmdata writer, pulseaudio and puredata
---------------------------------------

$ rm /tmp/pd_testshm ; padsp pd -oss 1.pd
$ gst-launch -v shmsrc is-live=true socket-path=/tmp/pd_testshm ! gdpdepay ! decodebin2 ! pulsesink sync=false


shmdata writer, jack and puredata
---------------------------------
$/usr/bin/jackd -r -dalsa -dhw:1,0 -r44100 -p64 -n2 -Xseq
$ rm /tmp/pd_testshm ; pd -jack -blocksize 64 1.pd
$ gst-launch -v shmsrc is-live=true socket-path=/tmp/pd_testshm ! gdpdepay ! decodebin2 ! jackaudiosink sync=false

(pd blocksize must be equal or higher than jack blocksize)

shmdata writer and reader
-------------------------
you can read your shmdata with shmsrc~
$ rm /tmp/pd_testshm ; padsp pd  -blocksize 512 -oss 2.pd

shmdata path
------------
TODO 


writing code for a writer (libshmdata-any)
------------------------------------------
$sudo make install
$cp examples/data-writer.c ~/tmp/
$cd ~/tmp
$gcc data-writer.c -o datawriter `pkg-config --cflags --libs shmdata-any-0.6`


writing code for a reader (libshmdata-any)
------------------------------------------
$sudo make install
$cp examples/data-reader.c ~/tmp/
$cd ~/tmp
$gcc data-reader.c -o datareader `pkg-config --cflags --libs shmdata-any-0.6`


switcher
========

install
-------

  $ sudo apt-get install automake bison build-essential flex libtool
  $ sudo apt-get install libglib2.0-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libjson-glib-dev liblo-dev
  $ ./autogen.sh
  $ ./configure
  $ make
  $ sudo make install
  $ sudo ldconfig

help
----
TODO 
switcher -h


create quiddity, invoke and remove quiddity
-------------------------------------------
$switcher -d --osc-port 7770
$rm /tmp/pd_testshm ; padsp pd -oss switcher-1.pd

register to switcher properties (get the logs)
----------------------------------------------
$switcher -d --osc-port 7770
$rm /tmp/pd_testshm ; padsp pd -oss switcher-2.pd

play Big Buck Bunny from the web 
--------------------------------
audio in pd, video in an external window

$switcher -d --osc-port 7770
$/usr/bin/jackd -r -dalsa -dhw:1,0 -r44100 -p64 -n2 -Xseq
$rm /tmp/pd_testshm ; pd -jack switcher-3.pd

stream to a location
--------------------

$switcher -d --osc-port 7770
$/usr/bin/jackd -r -dalsa -dhw:1,0 -r44100 -p64 -n2 -Xseq
$rm /tmp/pd_testshm ; pd -jack switcher-4.pd
