#!/bin/bash

# the script will fail if one command fails
set -e

gst-launch-1.0 -e --gst-plugin-path=/usr/local/lib/gstreamer-1.0/:/usr/lib/gstreamer-1.0/ videotestsrc pattern=18 ! shmdatasink socket-path=/tmp/check_shmshot &
gstpipe=$!

sleep 0.1

# non interactive mode
shmshot --shmpath /tmp/check_shmshot --folder /tmp/ --name check_shmshot_file

if [ -f /tmp/check_shmshot_file.jpg ]
then
    rm /tmp/check_shmshot_file.jpg
else
    exit 1
fi

kill -s SIGTERM $gstpipe
