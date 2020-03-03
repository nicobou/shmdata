#!/usr/bin/env python3

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

import sys
import pyquid
import time
import assert_exit_1


sw = pyquid.Switcher('urivid', debug=True)

# creating the window
winqrox = sw.create(type='glfwin', name='win')
winqrox2 = sw.create(type='glfwin', name='win2')
if not winqrox:
    winqrox = sw.create(type='dummysink', name='win')
if not winqrox2:
    winqrox2 = sw.create(type='dummysink', name='win2')

# creating a dummysink for audio
aqrox = sw.create(type='dummysink', name='asink')

assert winqrox
assert winqrox2
assert aqrox


# creating the uri decoder
uri = sw.create('urisrc', 'uri').quid()

# connecting quiddities
assert winqrox.quid().invoke('connect-quid', ['uri', 'image-0'])
assert winqrox2.quid().invoke('connect-quid', ['uri', 'video-0'])
assert aqrox.quid().invoke('connect-quid', ['uri', 'audio-0'])

assert uri.set('uri', 'https://gitlab.com/sat-metalab/switcher/raw/master/tests/oie.mp3')
time.sleep(2)

assert uri.set('uri', 'https://download.blender.org/peach/bigbuckbunny_movies/BigBuckBunny_320x180.mp4')
time.sleep(2)

assert uri.set(
    'uri', 'https://gitlab.com/sat-metalab/switcher/raw/master/doc/Switcher_horizontal_shadow_C.png')
time.sleep(2)

assert uri.set('uri', 'https://download.blender.org/peach/bigbuckbunny_movies/BigBuckBunny_640x360.m4v')
time.sleep(2)

assert uri.set(
    'uri', 'https://gitlab.com/sat-metalab/switcher/raw/master/doc/Switcher_horizontal_shadow_C.png')
time.sleep(2)

assert uri.set('uri', 'https://gitlab.com/sat-metalab/switcher/raw/master/doc/mapper_prop_osc.png')
time.sleep(2)
