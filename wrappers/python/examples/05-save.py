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

sw = pyquid.Switcher('save_example', debug=True)

# instantiate and use some quiddities
winqrox = sw.create(type='glfwin', name='win')
# The following replace the glfwin quiddity by a dummy quiddity if glfwin is not available
if None == winqrox:
    winqrox = sw.create(type='dummysink', name='win')
win = winqrox.quid()
vid = sw.create('videotestsrc', 'vid').quid()
assert win.invoke('connect-quid', ['vid', 'video'])
assert win.invoke('disconnect-all')

assert win.invoke('connect', [vid.make_shmpath('video')])
assert vid.set('started', True)

time.sleep(1)

# save the switcher state
state = sw.get_state()
assert not state.empty()
with open('save.switcher', 'w') as save_file:
    assert 0 < save_file.write(state.json())

# make another switcher
sw2 = pyquid.Switcher('pyQuidSave2')

# creating a quiddity that will not be affected by reloading
usage = sw2.create('systemusage')

# consider current state as initial state
sw2.reset_state(False)

# load the save file
with open('save.switcher', 'r') as save_file:
    content = save_file.read()
sw2.load_state(pyquid.InfoTree(content))

# check win and vid exist
assert None != sw.get_qrox_from_name('win')
assert None != sw.get_qrox_from_name('vid')

time.sleep(1)

total_mem = usage.quid().get_info('top.mem.total')
# system usage is still here
assert total_mem.isalnum()
