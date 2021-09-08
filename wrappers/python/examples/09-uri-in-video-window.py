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
try:
    win = sw.create(type='glfwin', nickname='win')
except RuntimeError:
    win = sw.create(type='dummysink', nickname='win')
try:
    win2 = sw.create(type='glfwin', nickname='win2')
except RuntimeError:
    win2 = sw.create(type='dummysink', nickname='win2')

# creating a dummysink for audio
a = sw.create(type='dummysink', nickname='asink')

# creating the uri decoder
uri = sw.create('urisrc', 'uri')

# connecting quiddities
assert win.invoke('connect-quid', [uri.id(), 'image-0'])
assert win2.invoke('connect-quid', [uri.id(), 'video-0'])
assert a.invoke('connect-quid', [uri.id(), 'audio-0'])

assert uri.set('uri', 'https://gitlab.com/sat-metalab/switcher/raw/master/tests/oie.mp3')
time.sleep(2)

assert uri.set('uri', 'https://download.blender.org/peach/bigbuckbunny_movies/BigBuckBunny_320x180.mp4')
time.sleep(2)

assert uri.set(
    'uri', 'https://gitlab.com/sat-metalab/switcher/raw/master/doc/logo/png/Switcher-color-horizontal-white-text.png')
time.sleep(2)

assert uri.set('uri', 'https://download.blender.org/peach/bigbuckbunny_movies/BigBuckBunny_640x360.m4v')
time.sleep(2)

assert uri.set(
    'uri', 'https://gitlab.com/sat-metalab/switcher/raw/master/doc/logo/png/Switcher-color-horizontal-white-text.png')
time.sleep(2)

assert uri.set('uri', 'https://gitlab.com/sat-metalab/switcher/raw/master/doc/mapper_prop_osc.png')
time.sleep(2)
