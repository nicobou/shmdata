#!/usr/bin/env python3

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

from pyquid import Switcher, Quiddity, WriterClaw, FollowerClaw
import assert_exit_1

sw = Switcher("claw-example", debug=True)

# create two "connection-quid" for testing. We use one as a shmdata writer, the other as a follower
wquid = sw.create("connection-quid")
fquid = sw.create("connection-quid")

# test the WriterClaw class
# =========================
wclaw = WriterClaw(wquid, "texture")
assert(wclaw.label() == "texture")
try:  # to get a non existing claw in order to check if an error is raised
    WriterClaw(wquid, "does-not-exist")
    assert(False)
except RuntimeError:
    pass
# check claw id is valid
assert(wclaw.id() != 0)
assert(wclaw.shmpath())
# check we have can_dos
can_dos = wclaw.get_can_do_str()
assert(len(can_dos) > 0)
for can_do in can_dos:
    assert(can_do)
# check reading connection specs
wspecs = wquid.get_connection_specs()
assert(not wspecs.empty())


# test the FollowerClaw class
# =========================
fclaw = FollowerClaw(fquid, "texture")
assert(fclaw.label() == "texture")
try:  # to get a non existing claw in order to check if an error is raised
    FollowerClaw(fquid, "does-not-exist")
    assert(False)
except RuntimeError:
    pass
# check claw id is valid
assert(fclaw.id() != 0)
# chech we can get claws from quiddities
wclaws = wquid.get_writer_claws()
assert(len(wclaws) > 0)
for item in wclaws:
    assert(item.id() != 0)
fclaws = fquid.get_follower_claws()
assert(len(fclaws) > 0)
for item in fclaws:
    assert(item.id() != 0)
# check reading connection specs
fspecs = fquid.get_connection_specs()
assert(not fspecs.empty())


# shmdata connections
# ===================

# connection to a WriterClaw
assert(fclaw.can_do_writer_claw(wquid, wclaw))
resclaw = fclaw.connect(wquid, wclaw)
# since flaclaw is not a meta follower, it should return itself
assert(resclaw == fclaw)
assert(resclaw.id() == fclaw.id())
assert(fclaw.disconnect())


# check we can get some can_do strings
can_dos = wclaw.get_can_do_str()
assert(len(can_dos) > 0)
# check first can_do is compatible for connection
assert(fclaw.can_do_shmtype_str(can_dos[0]))

# connection to a Quiddity

resclaw = fclaw.connect_quid(wquid)
# since flaclaw is not a meta follower, it should return itself
assert(resclaw == fclaw)
assert(resclaw.id() == fclaw.id())
assert(fclaw.disconnect())

# connection to a raw shmdata
resclaw = fclaw.connect_raw(wclaw.shmpath())
# since flaclaw is not a meta follower, it should return itself
assert(resclaw == fclaw)
assert(resclaw.id() == fclaw.id())
assert(fclaw.disconnect())


exit(0)
