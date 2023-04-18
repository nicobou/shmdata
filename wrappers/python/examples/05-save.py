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
dum = sw.create(kind='dummysink', nickname='dum')

vid = sw.create('videotestsrc', 'vid')
connection = dum.try_connect(vid)
assert connection.disconnect()

connection = dum.try_connect(vid)

assert vid.set('started', True)

time.sleep(1)

# save the switcher state
sw.session.save_as("test_state")

# make another switcher
sw2 = pyquid.Switcher('pyQuidSave2')

# creating a quiddity that will not be affected by reloading
prop = sw2.create('property-quid')

# consider current state as initial state
sw2.reset_state(False)

# load the save file
sw.session.load("test_state")

# check dum and vid exist
assert 0 != sw.get_quid_id('dum')
assert 0 != sw.get_quid_id('vid')

time.sleep(1)

val = prop.get("string_")
print(str(val))
# prop is still here
assert val.isalnum()

# remove test session file
sw.session.remove("test_state")
