#!/usr/bin/env python3
import pyquid
import time

sw = pyquid.Switcher('webrtc', debug=True)
w = sw.create("webrtc", "WebrtcClient")
w.quid().set("room", "RHtest")

vidqrox = sw.create('videotestsrc', 'vid')
vidqrox.quid().set("resolution", 5)
vidqrox.quid().set("started", True)

audqrox = sw.create('audiotestsrc', 'aud')
audqrox.quid().set('volume', '0.1')
audqrox.quid().set("wave", 6)
audqrox.quid().set("started", True)

wq = w.quid()
wq.invoke('connect-quid', ['vid', 'video'])
wq.invoke('connect-quid', ['aud', 'audio'])
time.sleep(1)
wq.set("started", True)

while(True):
    time.sleep(1)
