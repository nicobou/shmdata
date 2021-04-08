#!/usr/bin/env python3
import pyquid
import time

sw = pyquid.Switcher('webrtc', debug=True)
w = sw.create("webrtc", "WebrtcClient")
w.set("room", "RHtest")

vid = sw.create('videotestsrc', 'vid')
vid.set("resolution", 5)
vid.set("started", True)

aud = sw.create('audiotestsrc', 'aud')
aud.set('volume', '0.1')
aud.set("wave", 6)
aud.set("started", True)

w.invoke('connect-quid', [vid.id(), 'video'])
w.invoke('connect-quid', [aud.id(), 'audio'])
time.sleep(1)
w.set("started", True)

while(True):
    time.sleep(1)
