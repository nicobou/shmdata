#!/usr/bin/env python3
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# Information trees are composed of key/value nodes.
# They are (de)serializable and are used in switcher in order to:
# i) get information from quiddities' internal state,
# ii) save & load of switcher states,
# iii) introspect,
# iv) configure and
# v) attach user specific data to quiddities.
# This file illustrates the use of v).

import sys
import pyquid
import json

# first create a quiddity and get a reference to its internal user data tree. Note this
# user data tree is available for each kind of quiddity.
sw = pyquid.Switcher('infotree', debug=True)
emptyquid = sw.create('empty-quid')
utree = emptyquid.get_user_tree()
assert utree.empty()

# The default serialization format is JSON.
assert 'null' == utree.json()

# add some nodes in the tree
assert utree.graft('.my.long', 5)
assert utree.graft('.my.float', 1.2345)
assert utree.graft('.my.bool', True)
assert utree.graft('.my.string', 'My eye is ziped')

# some nodes can be bundled as array
assert utree.graft("my.array.1", 1)
assert utree.graft("my.array.2", 1)
assert utree.graft("my.array.3", 0)
assert utree.graft("my.array.4", 6)
assert utree.tag_as_array("my.array")
assert [1, 1, 0, 6] == json.loads(utree.json('my.array'))

# get value of nodes, using the path of a node from the root
assert 5 == utree.get('.my.long')
assert 1.2345 == utree.get('.my.float')
assert True == utree.get('my.bool')
assert 'My eye is ziped' == utree.get('my.string')

# some nodes do not have a value
assert None == utree.get('.my')

# we are still changing quiddity's internal tree
assert utree.json() == emptyquid.get_user_tree().json()

# you can copy a part of the tree
subtree_cpy = utree.copy('.my')
assert 5 == subtree_cpy.get('long')
assert 1.2345 == subtree_cpy.get('float')
assert True == subtree_cpy.get('bool')
assert 'My eye is ziped' == subtree_cpy.get('string')

# we can also copy the entire tree
utree_cpy = utree.copy()
assert utree.json() == utree_cpy.json()

# and remove some part
assert utree_cpy.prune('my.string')
assert utree.json() != utree_cpy.json()
assert not utree_cpy.prune('not.existing.branch')

# we can create our own info tree
from_default = pyquid.InfoTree()
assert 'null' == from_default.json()

# or create one from JSON
from_json = pyquid.InfoTree('{ "one" : 1.0 }')
assert 1.0 == from_json.get('one')

# and finally, you can graft a tree into a tree
utree.graft('other tree', from_json)
assert utree.json('other tree') == from_json.json()

# ensure parsing errors are handled
error_raised = False
try:
    pyquid.InfoTree('{ "missing quote : 1.0 }')
except RuntimeError as e:
    print('The error raised is: ', e)
    error_raised = True
assert error_raised
