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


# "xxx_user_data" will be passed to the subscribe method
width_user_data = 'width user data'
height_user_data = 'height user data'
my_width = 222
my_height = 123
num_success = 0


def on_property_changed(value, user_data):
    global num_success, width_user_data, height_user_data
    if (user_data == width_user_data):
        assert value == my_width
    if (user_data == height_user_data):
        assert value == my_height
    num_success += 1
    if (num_success == 4):
        exit(0)


sw = pyquid.Switcher("prop-sub", debug=True)

# create a quiddity
vid = sw.create("videotestsrc", "vid")

# check if the "width" property is available with this quiddity
assert "width" in pyquid.InfoTree(
    vid.get_info_tree_as_json(".property")).get_key_values('id', False)

# subscribe to the property named "width". Unsubscribe will be called with vid destruction
assert vid.subscribe("width", on_property_changed, width_user_data)
assert vid.subscribe("height", on_property_changed, height_user_data)

vid.set("height", my_height)
vid.set("width", my_width)
my_height += 1
my_width += 1
vid.set("height", my_height)
vid.set("width", my_width)


# wait for the "on_property_changed" callback to be triggered
time.sleep(1)

# the test will fail if the callback has not been invoked before
exit(1)
