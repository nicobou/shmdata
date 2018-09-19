#!/usr/bin/env python3

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

import math
import sys
import time
sys.path.insert(0, '/usr/local/lib/python3/dist-packages')
import pyshmdata

def cb(user_data, buffer, datatype, parsed_datatype):
    data = buffer.decode(encoding="utf-8")
    print(user_data, data, parsed_datatype)

data = ['all', 'your', 'base']

writer = pyshmdata.Writer(path="/tmp/some_shmdata", datatype="application/x-raw,fun=yes")
reader = pyshmdata.Reader(path="/tmp/some_shmdata", callback=cb, user_data=data)

start_time = time.clock_gettime(time.CLOCK_REALTIME)

while True:
    timestamp = time.clock_gettime(time.CLOCK_REALTIME) - start_time
    writer.push(buffer=bytearray("are belong to us", encoding="utf-8"), timestamp=math.floor(timestamp * 1e9))
    time.sleep(0.5)
