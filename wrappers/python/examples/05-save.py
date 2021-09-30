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


sw = pyquid.Switcher('save_example', debug=True)

# instantiate and use some quiddities
try:
    win = sw.create(kind='glfwin', nickname='win')
except RuntimeError:
    # The following replace the glfwin quiddity by a dummy quiddity if glfwin is not available
    win = sw.create(kind='dummysink', nickname='win')

vid = sw.create('videotestsrc', 'vid')
connection = win.try_connect(vid)
assert connection.disconnect()

connection = win.try_connect(vid)

assert vid.set('started', True)

time.sleep(1)

# save the switcher state
sw.session.save_as("test_state")

# make another switcher
sw2 = pyquid.Switcher('pyQuidSave2')

# creating a quiddity that will not be affected by reloading
usage = sw2.create('systemusage')

# consider current state as initial state
sw2.reset_state(False)

# load the save file
sw.session.read("test_state")

# check win and vid exist
assert 0 != sw.get_quid_id('win')
assert 0 != sw.get_quid_id('vid')

time.sleep(1)

total_mem = usage.get_info('top.mem.total')

# system usage is still here
assert total_mem.isalnum()

# remove test session file
sw.session.remove("test_state")
