#!/usr/bin/env python3

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

from .base import SocketIOTestCase
import time

# test quiddity data
videotest_quid = {
    "kind": "videotestsrc",
    "nickname": "test"
}

signal_quid = {
    "kind": "signal",
    "nickname": "signal"
}


class PropertyTestCase(SocketIOTestCase):
    """Test case for the `UserTree API`

    Available tests:
      - property.get
      - property.set
      - property.updated
    """

    # reverences on the created quiddities
    videotest_id = None

    def setUp(self):
        # creation of a videotestsrc quiddity
        data = tuple(videotest_quid.values())
        err, res = self.sio.call('quiddity.create', data=data)
        self.assertIsNone(err)
        self.videotest_id = res['id']

    def tearDown(self):
        err, res = self.sio.call('session.clear')
        self.videotest_id = None
        self.signal_id = None
        self.rcvd_data = []

    events = [
        'property.updated'
    ]

    def test_set_property(self):
        err, res = self.sio.call('property.set', data=(self.videotest_id, 'resolution', '2'))
        self.assertIsNone(err)
        self.assertIsInstance(res, str)
        self.assertEqual(res, '2')

    def test_get_property(self):
        err, res = self.sio.call('property.get', data=(self.videotest_id, 'resolution'))
        self.assertIsNone(err)
        self.assertIsInstance(res, str)
        self.assertEqual(res, '1')

    def test_property_updated(self):
        err, res = self.sio.call('property.set', data=(self.videotest_id, 'resolution', '2'))
        time.sleep(1)
        event, grafted_data = self.rcvd_data.pop()
        self.assertEqual(event, 'property.updated')
        print(grafted_data)
        id, propName, value, propId = grafted_data
        self.assertEqual(id, self.videotest_id)
        self.assertEqual(propName, 'resolution')
        self.assertEqual(value, '2')
