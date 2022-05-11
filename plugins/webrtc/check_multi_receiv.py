#!/usr/bin/env python3

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

#  dummy    dummy    dummy    dummy
#    ▲         ▲     ▲         ▲
#    │         │     │         │
#    └─────── receiver ────────┘
#              ▲   ▲
#     ┌────────┘   └──────────┐
#     │                       │
# sender 1                sender 2
#   ▲   ▲                  ▲  ▲
#   │   └────────────────┐ │  │
#   │                    │ │  │
#   │   ┌────────────────┼─┘  │
#    aud                   vid

# This tests creates the above configuration and ensure the "receiver"
# quiddity ouputs shmdata for the all for received streams, checking
# the four "dummysink" quiddities receive frames (with
# "frame_recieved" property)

import asyncio
import os
import pyquid
import sys
import time

from dataclasses import dataclass
from multiprocessing import Process

from signaling.simple_server import WebRTCSimpleServer
from util.generator import Generator

dummyquids = []

# 0. create the switcher

sw = pyquid.Switcher('multireceiv', debug=True)
assert 'multireceiv' == sw.name()

# 1. create, connect and start quiddities

# media quids
vid = sw.create(kind='videotestsrc', nickname='vid')
assert None != vid
audio = sw.create(kind='audiotestsrc', nickname='audio')
assert None != audio

# create a webrtc quiddities
sender1 = sw.create(kind='webrtc', nickname='sender1')
assert None != sender1
assert sender1.set('username', sender1.nickname())
sender2 = sw.create(kind='webrtc', nickname='sender2')
assert None != sender2
assert sender2.set('username', sender2.nickname())
receiver = sw.create(kind='webrtc', nickname='receiver')
assert None != receiver
assert receiver.set('username', receiver.nickname())

# connect shmdatas
assert sender1.try_connect(vid)
assert sender1.try_connect(audio)
assert sender2.try_connect(vid)
assert sender2.try_connect(audio)
assert receiver.try_connect(vid)
assert receiver.try_connect(audio)


def on_receiver_new_shmdata(data, receiv_conspec):
    # track new shmdata creation from the 'receive' quiddity and connect to a newly created dummy quiddity
    global sw, receiver
    receiv_conspec = receiver.get_connection_specs()
    print('notified of a new shmdata for ' + receiv_conspec.get(data.get() + '.label'))
    print('shmdata writer id is ' + str(receiv_conspec.get(data.get() + '.swid')))
    quid = sw.create(kind='dummysink')
    assert pyquid.FollowerClaw(quid, "default").connect(
        receiver,
        pyquid.WriterClaw(receiver, receiv_conspec.get(data.get() + '.label')))
    dummyquids.append(quid)


assert(receiver.subscribe("on-connection-spec-added", on_receiver_new_shmdata, None))

# start
assert vid.set('started', True)
assert audio.set('started', True)
time.sleep(1)

# 2. Name the room to join

assert sender1.set('room', 'switcher-room')
assert sender2.set('room', 'switcher-room')
assert receiver.set('room', 'switcher-room')

# 3. start the webrtc server


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

# 4. join the room
assert sender1.set('started', True)
assert sender2.set('started', True)
assert receiver.set('started', True)

time.sleep(2)

# check shmdatas has been created
assert len(dummyquids) == 4
for quid in dummyquids:
    assert(quid.get('frame-received'))

server_process.terminate()

# 5. notify success with exit value
exit(0)
