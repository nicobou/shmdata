#!/usr/bin/env python3

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

import time
import pyquid


on_created_success = False
on_removed_success = True
on_created_id = 0
on_removed_id = 0


def on_created_cb(id, user_data):
    global on_created_id
    assert user_data == my_user_data
    on_created_id = id


def on_removed_cb(id, user_data):
    global on_removed_id
    assert user_data == my_user_data
    on_removed_id = id


sw = pyquid.Switcher("switcher-signals", debug=True)

my_user_data = {"test": {"test2": True}}

# subscribe to the 'on-quiddity-created' signal
assert sw.subscribe("on-quiddity-created", on_created_cb, my_user_data)

# create a quiddity
vid = sw.create("videotestsrc")
assert vid.id() == on_created_id

# wait for the signal to arrive,
time.sleep(0.5)

# unsubscribe from the 'on-quiddity-created' signal
assert sw.unsubscribe("on-quiddity-created")

# create a quiddity (no callback should be fired)
vid2 = sw.create("videotestsrc")

# subscribe to the 'on-quiddity-removed' signal
assert sw.subscribe("on-quiddity-removed", on_removed_cb, my_user_data)

# remove a quiddity
vid2_id = vid2.id()
assert sw.remove(vid2_id)
assert vid2_id == on_removed_id

# wait for the signal to arrive,
time.sleep(0.5)

# unsubscribe from the 'on-quiddity-removed' signal
assert sw.unsubscribe("on-quiddity-removed")

# remove a quiddity (no callback should be fired)
assert sw.remove(vid.id())

exit(0)
