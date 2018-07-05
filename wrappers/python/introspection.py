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

sw = pyquid.Switcher(name="pyquid")

print("list classes names:")
for classname in sw.list_classes():
    print(classname)


print("classes doc:")
print(sw.classes_doc())

print("class doc for videotestsrc:")
print(sw.class_doc("videotestsrc"))

print("quiddity introspection")
sw.create("videotestsrc")
atest = sw.create("audiotestsrc", "atest")

print("list quiddities")
print(sw.list_quids())

print("description of quiddities")
print(sw.quids_descr())

print("description of atest")
print(sw.quid_descr(atest.id()))
