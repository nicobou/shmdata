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

sw = pyquid.Switcher(name="pyQuidSave")

# instanciate and use some quiddities
win = sw.create("glfwin", "win").quid()
vid = sw.create("videotestsrc", "vid").quid()
vid.set_str_str("started", "true")
win.invoke_str("connect", [vid.make_shmpath("video")])

time.sleep(2)

# save the switcher state
state = sw.get_state()
with open('save.switcher', 'w') as save_file:
    save_file.write(state.json())

# make another swicher
sw2 = pyquid.Switcher(name="pyQuidSave2")

# creating a quiddity that will not be affected by reloading
usage = sw2.create("systemusage")

# consider current state as initial state
sw2.reset_state(clear=False)

# load the save file
with open('save.switcher', 'r') as save_file:
    content = save_file.read()
    sw2.load_state(pyquid.InfoTree(json=content))
    print(pyquid.InfoTree(json=content).json())

time.sleep(2)

# system usage is still here
print("system usage is still here, and total system memory is : " +
      str(usage.quid().get_info("top.mem.total")))
