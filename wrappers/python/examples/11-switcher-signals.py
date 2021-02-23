#!/usr/bin/env python3

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

import pyquid
import time

success = False


def on_create_remove_cb(name, user_data):
    global success
    assert user_data == my_user_data
    assert name == my_name
    # success
    success = True


sw = pyquid.Switcher("switcher-signals", debug=True)

my_user_data = {"test": {"test2": True}}
my_name = "vid1"

# subscribe to the 'on-quiddity-created' signal
assert sw.subscribe("on-quiddity-created", on_create_remove_cb, my_user_data)

# create a quiddity
qroxvid = sw.create("videotestsrc", my_name)
assert None != qroxvid

# wait for the signal to arrive,
time.sleep(0.5)

# unsubscribe from the 'on-quiddity-created' signal
assert sw.unsubscribe("on-quiddity-created")

# create a quiddity (no callback should be fired)
qroxvid2 = sw.create("videotestsrc", "vid2")
assert None != qroxvid2

# subscribe to the 'on-quiddity-removed' signal
assert sw.subscribe("on-quiddity-removed", on_create_remove_cb, my_user_data)

# remove a quiddity
assert sw.remove(qroxvid.id())

# wait for the signal to arrive,
time.sleep(0.5)

# unsubscribe from the 'on-quiddity-removed' signal
assert sw.unsubscribe("on-quiddity-removed")

# remove a quiddity (no callback should be fired)
assert sw.remove(qroxvid2.id())

if (not success):
    exit(1)
exit(0)
