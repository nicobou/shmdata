Script with OSC Quiddities
=======

Switcher can transmit OSC messages along with other streams. This is done through OSC to Shmdata and Shmdata to OSC conversion. Here we give an example with the liblo tools

## Install liblo-tools

```
sudo apt install liblo-tools
```

## OSC to Shmdata

In a terminal, run switcher:
```
switcher -d
```

In another terminal, send command for creating OSC to Shmdata quiddity (named OSCreceiver):
```
switcher-ctrl -C OSCsrc OSCreceiver
switcher-ctrl -s OSCreceiver port 23456
switcher-ctrl -s OSCreceiver started true
```

In another terminal, check if frames are coming into the OSCreceiver Shmdata.
```
sdflow $(switcher-ctrl -p OSCreceiver osc)
```
Note: if a message is received by OSCreceiver, then sdflow command will display a line like the following:
```
0    size: 20    data: 0000662C000000002F656C706D6173...
```

Send an OSC message using liblo command line tools:
```
oscsend localhost 23456 /sample/ f 3.14
```
Then you should have seen the frame displayed by the sdflow command.

## Shmdata to OSC

(This assumes OSC to Shmdata is still running.)

```
OSCSINK_ID=$(switcher-ctrl -C OSCsink OSCsender)
switcher-ctrl -s OSCsender port 2397
switcher-ctrl -s OSCsender host localhost
switcher-ctrl -i OSCsender connect-quid $OSCSINK_ID osc
```

Run an OSC receiver using liblo command line tools:
```
oscdump 2397
```

And send an OSC message to OSC Receiver:
```
oscsend localhost 23456 /sample/ f 3.14
```

oscdump should display a line like the following:
```
de5529cb.3b610279 /sample/ f 3.140000

```

The message was received and written to a Shmdata by OSCreceiver. OSCsend did read the Shmdata and sent the message to localhost, port 2397.
