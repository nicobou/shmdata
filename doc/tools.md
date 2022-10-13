Switcher includes some tools under `tools` directory.

[[_TOC_]]

## Shmdata2jack

A script that connect to an audio Shmdata and publish it to jack audio connection kit, as a jack client.

### Usage

```sh
shmdata2jack -h
usage: shmdata2jack [-h] [-v] [-d] [-n NAME] [-s SERVER] [--no-auto-connect] [--connect-all-to-first]
                    [--connect-only-first] [--do-not-convert-format] [--do-not-convert-rate]
                    [--connect-to CONNECT_TO] [--connection-index CONNECTION_INDEX]
                    shmpath

positional arguments:
  shmpath               the audio shmdata to be sent to jack

options:
  -h, --help            show this help message and exit
  -v, --verbose         increase output verbosity
  -d, --debug           print switcher log messages
  -n NAME, --name NAME  jack client name
  -s SERVER, --server SERVER
                        jack server name
  --no-auto-connect     do not connect to another jack client
  --connect-all-to-first
                        connect all channels to the first
  --connect-only-first  connect only first channel
  --do-not-convert-format
                        do not try to convert sample format (required if shmdata has more than 63 channels)
  --do-not-convert-rate
                        do not try to convert sample rate (required if shmdata has more than 63 channels)
  --connect-to CONNECT_TO
                        name of the jack client to connect to (default is system:playback_%d). Note that %d will be
                        replaced by the channel index.
  --connection-index CONNECTION_INDEX
                        start connecting to other client from this channel index (default is 1)
```

## Image snapshot from a video Shmdata

The `shmshot` command takes image snapshot(s) from a v4l2 stream or a video shmdata stream and exit. Additionaly, an interactive mode allows a user to trigger snapshots and obtain a sequence of images.

### Usage

```sh
./shmshot --help
usage: shmshot [-h] [-s SHMPATH] [-c CAPTURE_DEVICE] [-d] [-l] [-f FOLDER] [-n NAME] [-i] [-v]

  Shmshot takes image snapshot(s) from a video shmdata stream and exit. Additionaly, an interactive mode allows a user to trigger snapshots and obtain a sequence of images. Shmshot can read from a video shmdata (using the --shmpath option) or read from a v4l2 device (using the --capture-device option). By default, shmshot with read the default v4l2 capture device available.

  Note that a video shmdata can be created using the following command: 
gst-launch videotestsrc pattern=18 ! shmdatasink socket-path=/tmp/video_shm

  In interactive mode (--interactive option), image snapshots are taken only when the user hit the space bar. Press escape (or ctrl-c) in order to quit. In interactive mode, you can display a the source video in a preview window (--preview option).

"Cause you can't, you won't and you don't stop."

Examples:
========
Get an image snapshot from default camera in current folder:
shmshot

Get an image from a video shmdata located at '/tmp/video_shm':
shmshot -s /tmp/video_shm

Save the image in a specific directory with a specific name
shmshot --folder ~/Picture --name me_as_bytes # writes '/tmp/me_as_bytes.jpg'

Run shmshot in interactive mode with video preview and image written to '~/tmp/'
shmshot -i -v -f ~/tmp/

Arguments:
=========

options:
  -h, --help            show this help message and exit
  -s SHMPATH, --shmpath SHMPATH
                        shmpath of the video stream to capture
  -c CAPTURE_DEVICE, --capture-device CAPTURE_DEVICE
                        video capture device (name or number, as displayed with option -l)
  -d, --debug           print switcher log messages
  -l, --list-camera     list available cameras
  -f FOLDER, --folder FOLDER
                        folder where images with be stored
  -n NAME, --name NAME  Name of the jpeg files to be produced. You can use printf format for numbering files. The default name for non interactive capture is 'shmshot_YYYY-MM-DD_HH:MM:SS_NSEC.jpg'.
  -i, --interactive     Run in interactive mode
  -v, --video-preview   Display the video shmdata in a window
```

## Display a video from a capture device

The `swcam-display` command allow to display a video stream from a capture device.

### Usage

```sh
./swcam-display --help
usage: swcam-display [-h] [-l] [-d] [-c CAPTURE_DEVICE]

options:
  -h, --help            show this help message and exit
  -l, --list-camera     list available cameras
  -d, --debug           print switcher log messages
  -c CAPTURE_DEVICE, --capture-device CAPTURE_DEVICE
                        video capture device (name or number, as displayed with option -l)
```

## Request informations about installed Quiddities

The `swquid-info` command provides information about available quiddities: properties, methods, signals, claw, infotree and description. 

### Usage

```sh
swquid-info --help
usage: swquid-info [-h] [-a] [-p] [-m] [-s] [-w] [-i] [-l] [-b SET_BEFORE [SET_BEFORE ...]] [-k] [-d] [-v]
                   [quiddity_kind]

Get information about available quiddities: properties, methods, signals, claw, infotree and description.
The result is json formatted, except for list-all (-a) and short-list (-l) options that provide only a list.

Note the "short-line" option (-l) is only valid when combined with one of p, m or s options.

Examples:
========
For the list of all quiddities installed on your system:
swquid-info -a

To list all properties available at startup for the videotestsrc quiddity:
swquid-info -lp videotestsrc

Display full documentation of the "width" property of a videotestsrc quiddity:
swquid-info -p videotestsrc.width

Display description of the "width" property of a videotestsrc quiddity:
swquid-info -p videotestsrc.width.description

Display Shmdata writer information from the quiddity infotree after setting resolution and started properties:
swquid-info -b resolution 3 -b started true -i videotestsrc.shmdata.writer

Arguments:
=========

positional arguments:
  quiddity_kind         You can add method or property or signal id for a shorter output

options:
  -h, --help            show this help message and exit
  -a, --list-all        list all quiddity kinds
  -p, --properties      list kind properties after initialization
  -m, --methods         list kind methods after initialization
  -s, --signals         list kind signals after initialization
  -w, --claws           list information about shmdata possible connection
  -i, --infotree        print the quiddity infotree, optionally with a branch path
  -l, --short-list      print only the list of properties, methods or signals.
  -b SET_BEFORE [SET_BEFORE ...], --set-before SET_BEFORE [SET_BEFORE ...]
                        set a property before the request (-b prop value)
  -k, --kind-doc        print kind documentation
  -d, --debug           print debug log
  -v, --version         print switcher version
```

## Switcher SIP reflector

A standalone Switcher service that registers to SIP server and waits for a call. When receiving a call, it automatically calls back caller with its own media. It effectively serves as a SIP loopback.

### Usage

```sh
swsip-reflector --help
usage: swsip-reflector [-h] [-s SERVER] [-p PORT] [-c CLIENT_PORT] [-u USER] [-P PASSWORD] [-v] {} ...

A Switcher SIP reflector service.

optional arguments:
  -h, --help            show this help message and exit
  -s SERVER, --server SERVER
                        SIP server hostname (defaults to localhost)
  -p PORT, --port PORT  SIP server port (defaults to 5060)
  -c CLIENT_PORT, --client_port CLIENT_PORT
                        SIP client port (defaults to 5060)
  -u USER, --user USER  SIP client user (defaults to user)
  -P PASSWORD, --password PASSWORD
                        SIP client password (defaults to password)
  -v, --verbose         print verbose output

Available Commands::
  {}

Use "sip-reflector [command] --help" for more information about a command.
```

Start service by providing server, SIP user and password to register it.

```sh
tools/swsip-reflector --server sip.domain.tld --user sip-user --password sip-password --verbose
```
