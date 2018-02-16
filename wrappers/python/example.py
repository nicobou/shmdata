#!/usr/bin/env python3
import math
import sys
import time
sys.path.insert(0, '/usr/local/lib/python3/dist-packages')
import pyshmdata

def cb(user_data, buffer, datatype):
    data = buffer.decode(encoding="utf-8")
    print(user_data, data)

data = ['all', 'your', 'base']

writer = pyshmdata.Writer(path="/tmp/some_shmdata", datatype="application/x-raw")
reader = pyshmdata.Reader(path="/tmp/some_shmdata", callback=cb, user_data=data)

start_time = time.clock_gettime(time.CLOCK_REALTIME)

while True:
    timestamp = time.clock_gettime(time.CLOCK_REALTIME) - start_time
    writer.push(buffer=bytearray("are belong to us", encoding="utf-8"), timestamp=math.floor(timestamp * 1e9))
    time.sleep(0.5)
