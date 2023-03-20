#!/bin/bash

# the script will fail if one command fails
set -e

# run dummy jack in background 
jackd --no-realtime --name check_shmdata2jack -d dummy -r 48000 & jackserv=$!

# produce an audio shmdata in background
gst-launch-1.0 --gst-plugin-path=/usr/local/lib/gstreamer-1.0/:/usr/lib/gstreamer-1.0/ audiotestsrc ! shmdatasink socket-path=/tmp/check_shmdata2jack & gstpipe=$!

# run shmdata2jack
shmdata2jack -v -s check_shmdata2jack -n check_shmdata2jack_client  --connect-all-to-first /tmp/check_shmdata2jack & shmdata2jack=$!

# wait
sleep 1

# kill processes
kill -s SIGTERM $shmdata2jack
kill -s SIGTERM $gstpipe
kill -s SIGTERM $jackserv

