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


def on_tree(key, quid):
    quid.get_info_tree_as_json(key.strip('\"'))


sw = pyquid.Switcher('quid-info-tree-stress', debug=True)

vquids = []
aquids = []
for i in range(1, 20):
    vquids.append(sw.create('videotestsrc', 'vid' + str(i)).quid())
    vquids[-1].subscribe('on-tree-grafted', on_tree, vquids[-1])
    vquids[-1].set('started', True)
    aquids.append(sw.create('audiotestsrc', 'aud' + str(i)).quid())
    aquids[-1].set('started', True)

for q in aquids:
    q.subscribe('on-tree-grafted', on_tree, q)


now = time.monotonic()
while (time.monotonic() < now + 4):
    for name in sw.list_quids():
        sw.get_qrox_from_name(name).quid().get_info_tree_as_json()
