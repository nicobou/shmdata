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

test_session_content = """
{
    "quiddities" : {
      "test" : "property-quid"
    },
    "nicknames" : {
      "test" : "test"
    },
    "properties" : {
      "test" : {
        "bool_" : false,
        "int_" : 1056,
        "string_" : "localhost"
      }
    }
  }
"""

# create a switcher.
sw = Switcher('pyquid', debug=True)

assert sw.session.write(test_session_content, 'test_session')

assert test_session_content == sw.session.read('test_session')

assert sw.session.load('test_session')
time.sleep(1)
assert len(sw.quiddities) == 1

assert sw.session.remove('test_session')
