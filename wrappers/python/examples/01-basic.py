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
from pyquid import Switcher, Quiddity, InfoTree
import time
import assert_exit_1

# create a switcher.
sw = Switcher('pyquid', debug=True)
# the switcher instance has a name
assert 'pyquid' == sw.name()
# it has a version
assert '' != sw.version()

# with switcher, you can create different kinds of named Quiddities.
# In this case, glfwin is a quiddity kind that manage video window
try:
    win = Quiddity(switcher=sw, kind='glfwin', nickname='win')
except RuntimeError:
    # The following replace the glfwin quiddity by a dummy quiddity if glfwin is not available
    # note here the quiddity is constructed from the switcher create method. This is equivalent
    # to creation from the pyquid.Quiddity constructor
    win = sw.create(kind='dummysink', nickname='win')

# creating a video source that will eventually be connected to the video window
vid = Quiddity(sw, 'videotestsrc', 'vid')

# Quiddities have nicknames & kinds
assert None != vid.nickname()
assert None != vid.get_kind()
nick = 'my vid'
assert vid.set_nickname(nick)
assert nick == vid.nickname()

# Quiddities have properties that can be accessed with get and set.
# Before action, you can get property information from the quiddity information tree
assert None != vid.get_info_tree_as_json('.property')
# you can save a copy into a InfoTree (see the infotree.py example for more details about this object)
propdoc = InfoTree(vid.get_info_tree_as_json('.property'))
assert not propdoc.empty()
# and ensure the win quiddity has a property with an id "started"
assert 'started' in propdoc.get_key_values("id", False)
assert False == vid.get('started')
# the video needs to be activated
assert vid.set('started', True)

# Quiddities can connect to other through shmdata
sfid = win.try_connect(vid)

time.sleep(1)

# win can disconnect from vid
assert sfid.disconnect()

time.sleep(1)

# and can reconnect
win.try_connect(vid)

time.sleep(1)

sw.remove(vid.id())
sw.remove(win.id())
