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
import time
import pyshmdata
import assert_exit_1

success = False


def cb(user_data, buffer, datatype, parsed_datatype):
    global success
    data = buffer.decode(encoding="utf-8")
    print(user_data, data, parsed_datatype)
    # we have been notified, success !
    success = True


data = ['all', 'your', 'base']

reader = pyshmdata.Reader(path="/tmp/some_shmdata", callback=cb, user_data=data)
writer = pyshmdata.Writer(path="/tmp/some_shmdata", datatype="application/x-raw,fun=yes")

start_time = time.monotonic()
while (not success and time.monotonic() < start_time + 4):
    writer.push(buffer=bytearray("are belong to us", encoding="utf-8"))

reader = None
writer = None
if not success:
    exit(1)
exit(0)
