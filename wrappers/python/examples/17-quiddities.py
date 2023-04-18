#!/usr/bin/env python3

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

import time
from pyquid import Switcher, Quiddity

# create a switcher.
sw = Switcher('pyquid', debug=True)

vid1 = sw.create('videotestsrc', 'vid1')
assert(sw.quiddities[0].get_kind() == 'videotestsrc')
vid2 = sw.create('videotestsrc', 'vid2')
assert(sw.quiddities[1].nickname() == 'vid2')
empty_quid = sw.create('empty-quid', 'empty')

assert len(sw.quiddities) == 3

for quid in sw.quiddities:
    assert isinstance(quid, Quiddity) is True

sw.remove(empty_quid.id())
assert len(sw.quiddities) == 2

sw.session.save_as('test_session')

sw.reset_state()
assert len(sw.quiddities) == 0

sw.session.load('test_session')
time.sleep(1)
assert len(sw.list_ids()) == 2
assert len(sw.quiddities) == 2
