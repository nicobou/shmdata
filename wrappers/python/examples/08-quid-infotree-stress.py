#!/usr/bin/env python3
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

import asyncio
import sys
import pyquid
import time
from typing import List
import assert_exit_1

queue = asyncio.Queue()
vquids: List[pyquid.Quiddity] = []
aquids: List[pyquid.Quiddity] = []


async def worker(queue: asyncio.Queue) -> None:
    while True:
        key, quid = await queue.get()
        print(key + '-- ' + quid.get_info_tree_as_json(key.strip('\"')))
        # Notify the queue that the "work item" has been processed.
        queue.task_done()


async def make_switcher_scenario() -> None:
    sw = pyquid.Switcher('quid-info-tree-stress', debug=True)

    for i in range(1, 10):
        vquids.append(sw.create('videotestsrc', 'vid' + str(i)).quid())
        vquids[-1].subscribe('on-tree-grafted', on_tree, vquids[-1])
        vquids[-1].set('started', True)
        vquids.append(sw.create('dummysink', 'vsink' + str(i)).quid())
        vquids[-1].subscribe('on-tree-grafted', on_tree, vquids[-1])
        vquids[-1].invoke('connect-quid', ['vid' + str(i), 'video'])
        aquids.append(sw.create('audiotestsrc', 'aud' + str(i)).quid())
        aquids[-1].set('started', True)
        aquids.append(sw.create('dummysink', 'asink' + str(i)).quid())
        aquids[-1].invoke('connect-quid', ['vid' + str(i), 'video'])

    for q in aquids:
        q.subscribe('on-tree-grafted', on_tree, vquids[-1])

    now = time.monotonic()
    while (time.monotonic() < now + 4):
        await asyncio.sleep(0.1)
        for name in sw.list_quids():
            sw.get_qrox_from_name(name).quid().get_info_tree_as_json()


def on_tree(key: str, quid: pyquid.Quiddity) -> None:
    queue.put_nowait((key, quid))


async def main():

    task = asyncio.ensure_future(worker(queue))

    init_task = asyncio.ensure_future(make_switcher_scenario())

    # waiting for end of work
    await asyncio.gather(init_task, return_exceptions=True)
    await queue.join()
    task.cancel()
    await asyncio.gather(task, return_exceptions=True)


loop = asyncio.get_event_loop()
loop.run_until_complete(main())
loop.close()
