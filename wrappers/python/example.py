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


def on_tree_grafted(key):
    print("Tree grafted at : " + key)


sw = pyquid.Switcher(name="pyquid")

print(sw.name())
print(sw.version())

# creating two quiddity
qrox = sw.create(type="glfwin", name="win")
quid = qrox.quid()

qroxvid = sw.create(type="videotestsrc", name="vid")
vid = qroxvid.quid()

# set get property
quid.get_str_str("height")
quid.set_str_str("height", "1024")

if not vid.subscribe("on-tree-grafted", on_tree_grafted):
    print("issue with subscribe")

vid.set_str_str("started", "true")

# connecting win to vid
quid.invoke_str("connect", [vid.make_shmpath("video")])

time.sleep(2)

quid.invoke_str("disconnect-all")

if not vid.unsubscribe("on-tree-grafted"):
    print("issue with unsubscribe")
else:
    print("unsubscribed")

quid.invoke_str("connect", [vid.make_shmpath("video")])

utree = quid.get_user_tree()
utree.graft(".my.long", 5)
utree.graft(".my.float", 1.2345)
utree.graft(".my.bool", True)
utree.graft(".my.string", "My eye is ziped")

print("utree: " + utree.json(".my."))

utree_cpy = utree.copy(".")

utree.prune("my")

print("utree: " + utree.json())

print("utree_cpy: " + utree_cpy.json())

print("long: " + str(utree_cpy.get("my.long")))
print("float: " + str(utree_cpy.get("my.float")))
if (utree_cpy.get("my.bool")):
    print("bool: true")
else:
    print("bool: false")
print("string: " + utree_cpy.get("my.string"))

print("json information tree: " + quid.get_info_as_json())

print("json information tree: " + quid.get_info_as_json(".property"))

print(".property.overlay_font_size.value: " + str(quid.get_info(".property.overlay_font_size.value")))

print(".property.brightness.value: " + str(quid.get_info(".property.brightness.value")))

print(".property.keyb_interaction.value: " + str(quid.get_info(".property.keyb_interaction.value")))

sw.remove(qrox.id())
