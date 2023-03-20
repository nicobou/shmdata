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


sw = pyquid.Switcher('urivid', debug=True)
audio_connection = None
image_connection = None


def on_con_spec_added(data, uri):
    global win, a, audio_connection, image_connection
    label = uri.get_connection_specs().get(data.get() + '.label')
    if 'audio-0' == label:
        audio_connection = a.try_connect(uri)
    else:
        image_connection = win.try_connect(uri)


def on_con_spec_removed(data, user_data):
    global audio_connection, image_connection
    label = uri.get_connection_specs().get(data.get() + '.label')
    if 'audio-0' == label:
        audio_connection.disconnect()
    else:
        image_connection.disconnect()


# creating the window
try:
    win = sw.create(kind='glfwin', nickname='win')
except RuntimeError:
    win = sw.create(kind='dummysink', nickname='win')


# creating a dummysink for audio
a = sw.create(kind='dummysink', nickname='asink')

# creating the uri decoder
uri = sw.create('urisrc', 'uri')

assert uri.subscribe("on-connection-spec-added", on_con_spec_added, uri)
assert uri.subscribe("on-connection-spec-removed", on_con_spec_removed, uri)

assert uri.set('uri', 'https://gitlab.com/nicobou/switcher/raw/master/tests/oie.mp3')
time.sleep(2)

assert uri.set(
    'uri', 'https://gitlab.com/nicobou/switcher/raw/master/doc/logo/png/Switcher-color-horizontal-white-text.png')
time.sleep(2)

assert uri.set('uri', 'https://download.blender.org/peach/bigbuckbunny_movies/BigBuckBunny_640x360.m4v')
time.sleep(2)

assert uri.set(
    'uri', 'https://gitlab.com/nicobou/switcher/raw/master/doc/logo/png/Switcher-color-horizontal-white-text.png')
time.sleep(2)

assert uri.set('uri', 'https://gitlab.com/nicobou/switcher/raw/master/doc/mapper_prop_osc.png')
time.sleep(2)
