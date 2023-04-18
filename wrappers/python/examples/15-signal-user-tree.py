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


success_graft = False
success_prune = False


def on_user_tree_grafted(data, user_data):
    global success_graft
    assert user_data == my_user_data
    assert not data.empty()
    # success
    success_graft = True


def on_user_tree_pruned(data, user_data):
    global success_prune
    assert user_data == my_user_data2
    assert not data.empty()
    # success
    success_prune = True


sw = pyquid.Switcher("userTreeSignal", debug=True)

# create a quiddity
dum = sw.create("empty-quid", "dum")

my_user_data = dum
my_user_data2 = str('swquid-info is my friend')

# check if on-tree-grafted is available with this quiddity
assert "on-user-data-grafted" in pyquid.InfoTree(
    dum.get_info_tree_as_json(".signal")).get_key_values('id', False)

# subscribe to a signal
assert dum.subscribe("on-user-data-grafted", on_user_tree_grafted, my_user_data)
assert dum.subscribe("on-user-data-pruned", on_user_tree_pruned, my_user_data2)

# access the user tree
utree = dum.get_user_tree()
branch_path = '.my,branch'

# graft and notify
assert utree.graft(branch_path, 'chocolat')
dum.notify_user_tree_grafted(branch_path)

# prune and notify
assert utree.prune(branch_path)
dum.notify_user_tree_pruned(branch_path)


# wait for the signal to arrive,
time.sleep(0.5)


if (not success_graft or not success_prune):
    exit(1)
exit(0)
