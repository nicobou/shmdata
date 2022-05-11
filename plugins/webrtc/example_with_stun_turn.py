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

sw1 = pyquid.Switcher('webrtc1', debug=True)
w1 = sw1.create("webrtc", "WebrtcClient1")
w1.set("signaling_server", "wss://dev.sip.scenic.sat.qc.ca:8443")
w1.set("stun_server", "stun://dev.sip.scenic.sat.qc.ca:3478")
#w1.set("turn_server", "turn://user:password@dev.sip.scenic.sat.qc.ca:3478")
w1.set("username", "switcher-client-1")
w1.set("room", "room-test")

vid1 = sw1.create('videotestsrc', 'vid1')
vid1.set("resolution", 5)
vid1.set("pattern", 18)
vid1.set("started", True)

aud1 = sw1.create('audiotestsrc', 'audio1')
aud1.set('volume', '0.6')
aud1.set("wave", 8)
aud1.set('started', True)

w1.try_connect(aud1)
w1.try_connect(vid1)
time.sleep(1)
w1.set('started', True)

sw2 = pyquid.Switcher('webrtc2', debug=True)
w2 = sw2.create("webrtc", "WebrtcClient2")
w2.set("signaling_server", "wss://dev.sip.scenic.sat.qc.ca:8443")
w2.set("stun_server", "stun://dev.sip.scenic.sat.qc.ca:3478")
#w2.set("turn_server", "turn://user:password@dev.sip.scenic.sat.qc.ca:3478")
w2.set("username", "switcher-client-2")
w2.set("room", "room-test")

vid2 = sw2.create('videotestsrc', 'vid2')
vid2.set("resolution", 5)
vid2.set("pattern", 18)
vid2.set("started", True)

aud2 = sw2.create('audiotestsrc', 'audio2')
aud2.set('volume', '0.6')
aud2.set("wave", 8)
aud2.set('started', True)

w2.try_connect(aud2)
w2.try_connect(vid2)
time.sleep(1)
w2.set('started', True)

while(True):
    time.sleep(1)
