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


sw = pyquid.Switcher("introspection", debug=True)

# list of kinds names
kind_list = sw.list_kinds()
assert 0 < len(kind_list)

# load custom kinds (bundles)
description = {
    "bundle": {
        "testBundle": {
            "pipeline": "property-quid name=Test",
            "doc": {
                "long_name": "Test",
                "category": "test",
                "tags": "writer",
                "description": "Test"
            }
        }
    }
}

sw.load_bundles(description)
kinds = sw.list_kinds()
assert "testBundle" in kinds

# kinds doc (JSON)
assert 0 < len(sw.kinds_doc())

# list kind doc per kind
for kind_name in kind_list:
    assert 0 < len(sw.kind_doc(kind_name))

# quiddity introspection
sw.create("videotestsrc")
atest = sw.create("audiotestsrc", "atest")

# check atest is in the list of quiddities
assert 'atest' in sw.list_quids()

# description of quiddities (JSON)
assert 0 < len(sw.quids_descr())

# description of atest
assert 0 < len(sw.quid_descr(atest.id()))
