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

spec_added_received = 0
spec_removed_received = 0


def on_con_spec_added(data, user_data):
    global spec_added_received
    spec_added_received += 1


def on_con_spec_removed(data, user_data):
    global spec_removed_received
    spec_removed_received += 1


sw = Switcher("dyn-claw-example", debug=True)

# create two dynamic shmdata quiddity for testing. We use one as a shmdata writer, the other as a follower
wquid = sw.create("dyn-writer-quid")
assert(wquid.subscribe("on-connection-spec-added", on_con_spec_added, None))
assert(wquid.subscribe("on-connection-spec-removed", on_con_spec_removed, None))

fquid = sw.create("dyn-reader-quid")
assert(fquid.subscribe("on-connection-spec-added", on_con_spec_added, None))
assert(fquid.subscribe("on-connection-spec-removed", on_con_spec_removed, None))

# chech we can get claws from quiddities
wclaws = wquid.get_writer_claws()
assert(len(wclaws) == 1)
# this dyn-writer-quid create two dynamic writers when the "video" property is set to True
wquid.set("video", True)
assert(spec_added_received == 2)
# we should get now two more WriterClaw now
wclaws = wquid.get_writer_claws()
assert(len(wclaws) == 3)

# check follower claws
fclaws = fquid.get_follower_claws()
assert(len(fclaws) == 1)

# test connection
try:
    # this must fail because it is not possible to connect to a meta writer claw
    fclaws[0].connect(wquid, wclaws[0])
    assert(False)
except RuntimeError:
    pass
# so connect to a newly created one
resclaw = fclaws[0].connect(wquid, wclaws[1])
# check the meta follower created a new non-meta claw
assert(fclaws[0].id() != resclaw.id())
# check the new claw creation has been notified
assert(spec_added_received == 3)
# check it is now part of the writer claw
assert(len(fquid.get_follower_claws()) == 2)

# now clean up everything
resclaw.disconnect()
wquid.set("video", False)
assert(spec_removed_received == 3)

exit(0)
