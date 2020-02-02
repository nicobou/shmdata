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

# "my_user_data" will be passed to the subscribe method
my_user_data = ["my", "user", "data"]
my_width = 123


def on_property_changed(value, user_data):
    assert value == my_width
    assert user_data == my_user_data
    # success
    exit(0)


sw = pyquid.Switcher("prop-sub", debug=True)

# create a quiddity
qroxvid = sw.create("videotestsrc", "vid")
assert None != qroxvid
vid = qroxvid.quid()

# check if the "width" property is available with this quiddity
assert "width" in pyquid.InfoTree(
    vid.get_info_tree_as_json(".property")).get_key_values('id', False)

# subscribe to the property named "width". Unsubscribe will be called with vid destruction
assert vid.subscribe("width", on_property_changed, my_user_data)

# WARNING: do not subscribe using the quid() method of a qrox. In this case subscription  is done on a temporary quid object which cannot hold data required when triggering the callback.
# The following is wrong and results in on_property_changed not being called back:
assert qroxvid.quid().subscribe("height", on_property_changed, my_user_data)

vid.set("width", my_width)


# wait for the "on_property_changed" callback to be triggered
time.sleep(1)

# the test will fail if the callback has not been invoked before
exit(1)
