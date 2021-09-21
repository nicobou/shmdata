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

success = False


def on_invocation_done(value, user_data):
    global success
    assert value == True
    assert user_data == my_user_data
    success = True


# create a switcher.
sw = pyquid.Switcher('pyquid', debug=True)

# creating a video source that will eventually be connected to the dummysink quiddity
vid = sw.create('videotestsrc', 'vid')

# the video needs to be activated
assert vid.set('started', True)

# creating a dummysink in order to illustrate invoke_async
dummysink = sw.create('dummysink', 'sink')
my_user_data = ["my", "user", "data"]
dummysink.invoke_async('connect-quid', [vid.id(), 'video'], on_invocation_done, my_user_data)

# wait 200 milliseconds
time.sleep(0.2)

if success == True:
    exit(0)
# fail if invocation done has not been notified
exit(1)
