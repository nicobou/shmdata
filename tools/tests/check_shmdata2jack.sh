#!/bin/bash

# the script will fail if one command fails
set -e

jackd --no-realtime --name check_shmdata2jack -d dummy -r 48000 &
jackserv=$!

gst-launch-1.0 --gst-plugin-path=/usr/local/lib/gstreamer-1.0/ audiotestsrc ! shmdatasink socket-path=/tmp/check_shmdata2jack &
gstpipe=$!

 JACK_DEFAULT_SERVER=check_shmdata2jack shmdata2jack -v -n check_shmdata2jack  --connect-all-to-first /tmp/check_shmdata2jack &
shmdata2jack=$!

sleep 1

kill -s SIGTERM $shmdata2jack
kill -s SIGTERM $gstpipe
kill -s SIGTERM $jackserv

