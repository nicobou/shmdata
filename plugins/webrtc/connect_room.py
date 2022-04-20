#!/usr/bin/env python3

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

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
