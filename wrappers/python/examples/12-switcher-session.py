#!/usr/bin/env python3

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

import pyquid
import os

sw = pyquid.Switcher('session_test', debug=True)

assert hasattr(sw, 'session')

# Save current session state to a new file under the directory
# defined by the `session.path` configuration key:
# ```json
# {
#   "session": {
#      "path": "path/to/session_directory"
#   }
# }
# ```
s1_filepath = sw.session.save_as("session1")
s2_filepath = sw.session.save_as("session2.json")

assert isinstance(s1_filepath, str) and os.path.exists(s1_filepath)
assert isinstance(s2_filepath, str) and os.path.exists(s2_filepath)

# Copy an existing session file to a new file
assert sw.session.copy("session2", "session3.json") is True

# list existing session files
session_files = sw.session.list()
assert isinstance(session_files, list) and len(session_files) >= 3

# load session file, parse it and load it into switcher's current state
assert sw.session.load("session1")
assert sw.session.load("session2.json")

# remove test session files
assert sw.session.remove("session1")
assert sw.session.remove("session2.json")
assert sw.session.remove("session3")

sw2 = pyquid.Switcher('session_test', debug=True)

assert (sw2.session != sw.session)

# check that session methods are restricted to the session files directory (XDG_CONFIG_HOME by default)
s3_filepath = sw.session.save_as('/../sneaky/path/to/s3.json')
assert isinstance(s3_filepath, str) and os.path.exists(s3_filepath)
assert "s3.json" in sw.session.list()
assert sw.session.copy('s3', '../s4')
assert "s4.json" in sw.session.list()
assert sw.session.remove('s3')
assert sw.session.load('../s4')
assert sw.session.remove('s4')

exit(0)
