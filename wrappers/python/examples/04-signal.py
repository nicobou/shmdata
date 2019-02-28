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

success = False


def on_tree_grafted(data, user_data):
    global success
    assert user_data == my_user_data
    assert 'null' != data
    # switcher signals provide a json serialized InfoTree,
    # so we need to strip 'data' in order to get the tree key
    # without double quotes at the end and at the begining
    assert 'null' != my_user_data.get_info_tree_as_json(data.strip('\"'))
    # success
    success = True


sw = pyquid.Switcher("signals", debug=True)

# create a quiddity
qroxvid = sw.create("videotestsrc", "vid")
assert None != qroxvid
vid = qroxvid.quid()

my_user_data = vid

# check if on-tree-grafted is available with this quiddity
assert "on-tree-grafted" in pyquid.InfoTree(
    vid.get_info_tree_as_json(".signal")).get_key_values('id', False)

# subscribe to a signal
assert vid.subscribe("on-tree-grafted", on_tree_grafted, my_user_data)

vid.set("started", True)

# wait for the signal to arrive,
time.sleep(0.5)

assert vid.unsubscribe("on-tree-grafted")

vid.set("started", False)

if (not success):
    exit(1)
exit(0)
