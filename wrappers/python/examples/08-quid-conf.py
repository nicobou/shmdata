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

success = True

# create a switcher.
sw = pyquid.Switcher('pyquid', debug=True)

# you can create two jack servers, one named "swcapture", then run this script and you
# will get a shmdata link between the two servers
# FIXME : make this script a proper cmake test with implementation
#         of a jack server quiddity and use of the jack dummy driver
swcapture = sw.create('jacksrc', 'swcapture', pyquid.InfoTree('{ "server_name" : "swcapture" }'))
swcapture.quid().set('started', True)

swcard = sw.create('jacksink', 'swcard')
swcard.quid().invoke('connect-quid', ['swcapture', 'audio'])

# wait 20 seconds
time.sleep(200)

if success == True:
    exit(0)
# fail if invocation done has not been notified
exit(1)
