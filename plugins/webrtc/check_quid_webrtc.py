#!/usr/bin/env python3

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

import asyncio
import os
import pyquid
import sys
import time

from dataclasses import dataclass
from multiprocessing import Process

from signaling.simple_server import WebRTCSimpleServer
from util.generator import Generator


success = False


def on_frame_received(data, user_data):
    global success
    success = True


# create a switcher.
sw = pyquid.Switcher('Webrtc', debug=True)
assert 'Webrtc' == sw.name()

# creation of this configuration:

# DummySink_1                 DummySink_2
#   |                            |
#   |                            |
# webRTC_1 -----audioquid------webRTC_2
#         |                    |
#         ------videoquid-------

# create a videotest quiddity
vid = sw.create(type='videotestsrc', nickname='vid')
assert None != vid

# create an audiotest quiddity
audio = sw.create(type='audiotestsrc', nickname='audio')
assert None != audio

# create a webrtc quiddity that manages a webrtcclient
web1 = sw.create(type='webrtc', nickname='webrtcclient1')
assert None != web1

# create a second webrtc quiddity
web2 = sw.create(type='webrtc', nickname='webrtcclient2')
assert None != web2

# create dummysinks for each webrtcclients
dummy1 = sw.create(type='dummysink', nickname='dummy1')
assert None != dummy1

dummy2 = sw.create(type='dummysink', nickname='dummy2')
assert None != dummy2

# connect audio and video through their shmpaths to the webrtc quids
vidshmpath = vid.make_shmpath('video')
audioshmpath = audio.make_shmpath('audio')

assert web1.invoke('connect', [vidshmpath])
assert web1.invoke('connect', [audioshmpath])

assert web2.invoke('connect', [vidshmpath])
assert web2.invoke('connect', [audioshmpath])

# connect the dummysink to the webrtc quids
assert dummy1.invoke('connect-quid', [web1.id(), 'webrtc'])
assert dummy2.invoke('connect-quid', [web2.id(), 'webrtc'])

# subscribe to the 'frame-received' property of the dummysinks
assert dummy1.subscribe('frame-received', on_frame_received, vid)
assert dummy2.subscribe('frame-received', on_frame_received, vid)

# configure and start the webrtc communication

# 1. start the video and audio quiddities
assert vid.set('started', True)
assert audio.set('started', True)
time.sleep(1)

# 2. Name the room to join
assert web1.set('room', 'switcher-room')
assert web2.set('room', 'switcher-room')

# 3. start the webrtc quid


def launch_server():
    @dataclass
    class Options:
        addr: str = ''
        port: int = 8443
        keepalive_timeout: int = 30
        cert_restart: bool = False
        cert_path: str = os.path.dirname(__file__)
        disable_ssl: bool = False
        health: str = '/health'

    loop = asyncio.get_event_loop()
    server = WebRTCSimpleServer(loop, Options())
    while True:
        server.run()
        loop.run_forever()


# to lauch the server, we need to generate a certificate first
cert = Generator()
file_dir = os.path.dirname(os.path.realpath(__file__))
cert.generate_certificate(key_file=os.path.join(file_dir, 'key.pem'),
                          cert_file=os.path.join(file_dir, 'cert.pem'))

# launch the server
server_process = Process(target=launch_server)
server_process.start()

time.sleep(0.2)

assert web1.set('started', True)
assert web2.set('started', True)

time.sleep(0.2)

server_process.terminate()

# exit
exit(success)
