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
import assert_exit_1

sw = pyquid.Switcher("introspection", debug=True)

# list of classes names
class_list = sw.list_classes()
assert 0 < len(class_list)

# load custom classes (bundles)
description = '''{
    "bundle": {
        "testBundle" : {
            "pipeline" : "dummy name=Test",
            "doc" : {
                "long_name" : "Test",
                "category" : "test",
                "tags" : "writer",
                "description" : "Test"
            }
        }
    }
}'''

sw.load_bundles(description)
classes = sw.list_classes()
assert "testBundle" in classes

# classes doc (JSON)
assert 0 < len(sw.classes_doc())

# list class doc per class
for class_name in class_list:
    assert 0 < len(sw.class_doc(class_name))

# quiddity introspection
sw.create("videotestsrc")
atest = sw.create("audiotestsrc", "atest")

# check atest is in the list of quiddities
assert 'atest' in sw.list_quids()

# description of quiddities (JSON)
assert 0 < len(sw.quids_descr())

# description of atest
assert 0 < len(sw.quid_descr(atest.id()))
