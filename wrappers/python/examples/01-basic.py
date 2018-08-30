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
sys.path.insert(0, '/usr/local/lib/python3/dist-packages')
import pyquid
import time
import assert_exit_1

# create a switcher.
sw = pyquid.Switcher('pyquid')
# the switcher instance has a name
assert 'pyquid' == sw.name()
# it has a version
assert '' != sw.version()

# with switcher, you can create different types of named Quiddities.
# In this case, glfwin is a quiddity type that manage video window
winqrox = sw.create(type='glfwin', name='win')
# a qrox is a handler to the internal quiddity, it provides access to
# the quiddity itself, its id and its name
assert None != winqrox
win = winqrox.quid()
# creating a video source that will eventually be connected to the video window
vidqrox = sw.create('videotestsrc', 'vid')
assert None != vidqrox
vid = vidqrox.quid()

# Quiddities have properties that can be accessed with get and set.
# Before action, you can get property information from the quiddity information tree
assert None != vid.get_info_tree_as_json('.property')
# you can save a copy into a InfoTree (see the infotree.py example for more details about this object)
propdoc = pyquid.InfoTree(vid.get_info_tree_as_json('.property'))
assert not propdoc.empty()
# and ensure the win quiddity has a property with an id "started"
assert 'started' in propdoc.get_key_values("id", False)
assert False == vid.get('started')
# the video needs to be activated
assert vid.set('started', True)

# Quiddities also have methods. For instance, win has a connect method
# connecting win to vid through its video shmpath
# (vid is a quiddity sharing a video stream through the shmdata library, and
# win is a quiddity that reads a video from a shmdata and displays it in a window)
# all quiddities provide the make_shmpath method, that give the shmdata path according to a keyword
# Usually, the keyword used is the type of media shared through the shmdata
vidshmpath = vid.make_shmpath('video')
assert win.invoke('connect', [vidshmpath])

time.sleep(1)

# win can disconnect from vid
assert win.invoke('disconnect', [vidshmpath])

time.sleep(1)
# and can reconnect, using the alternative connect-quid method
assert win.invoke('connect-quid', ['vid', 'video'])
time.sleep(1)

sw.remove(vidqrox.id())
sw.remove(winqrox.id())
