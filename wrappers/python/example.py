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

sw = pyquid.Switcher(name="pyquid")

print(sw.name())

#creating two quiddity
qrox = sw.create(type="glfwin", name="win")
quid = qrox.quid()

qroxvid = sw.create(type="videotestsrc", name="vid")
vid = qroxvid.quid()

# set get property
quid.get_str_str("height")
quid.set_str_str("height", "1024")
vid.set_str_str("started", "true")

# connecting win to vid
quid.invoke_str("connect", ["/tmp/switcher_pyquid_2_video"])
quid.invoke_str("disconnect-all", [])

quid.invoke_str("connect", [vid.make_shmpath("video")])



