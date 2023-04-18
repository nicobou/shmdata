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


class QuiddityTestCase(SocketIOTestCase):
    """Test case for the `Quiddity API`

    Available tests:
      - method.invoke
      - nickname.get
      - nickname.set
      - property.get
      - property.set
      - quiddity.connect
      - quiddity.create
      - quiddity.delete
      - quiddity.list
    """

    # registering some generic event callbacks for testing purposes
    events = [
        'quiddity.created',
        'quiddity.deleted',
        'nickname.updated'
    ]

    # test quiddity data
    test_quid = {
        "kind": "property-quid",
        "nickname": "test_name",
        "properties": {},
        "user_data": {"testData": True}
    }

    def test_01_quiddity_create(self):
        data = tuple(self.test_quid.values())
        err, res = self.sio.call('quiddity.create', data=data)
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, dict)

    def test_02_property_set(self):
        err, res = self.sio.call('quiddity.create', data=('videotestsrc', 'vid1'))
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, dict)
        data = (2, 'started', True)
        err, res = self.sio.call('property.set', data=data)
        time.sleep(1)
        self.assertIsNone(err)
        self.assertTrue(res)

    def test_03_property_get(self):
        err, res = self.sio.call('property.get', data=(2, 'started'))
        self.assertIsNone(err)
        self.assertTrue(res)

    def test_04_nickname_set(self):
        err, res = self.sio.call('nickname.set', data=(2, 'video1'))
        time.sleep(1)
        self.assertIsNone(err)
        self.assertTrue(res)
        event, updated_data = self.rcvd_data.pop()
        self.assertEqual('nickname.updated', event)
        self.assertIsInstance(updated_data[0], int)
        self.assertEqual(updated_data[1], "video1")

    def test_05_nickname_get(self):
        err, res = self.sio.call('nickname.get', data=2)
        time.sleep(1)
        self.assertIsNone(err)
        self.assertEqual(res, "video1")

    def test_06_quiddity_connect(self):
        err, res = self.sio.call('quiddity.create', data=('dummysink', 'win'))
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, dict)
        err, sfid = self.sio.call('quiddity.connect', data=(2, 3))
        self.assertIsNone(err)
        self.assertIsInstance(sfid, int)
        time.sleep(1)
        err, res = self.sio.call('quiddity.disconnect', data=(3, sfid))
        self.assertIsNone(err)
        self.assertTrue(res)

    def test_07_method_invokation(self):
        err, res = self.sio.call('quiddity.create', data=('method-quid', 'mquid'))
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, dict)
        err, res = self.sio.call('quiddity.invoke', data=(4, 'hello', ['Albert Camus']))
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, str)

    def test_08_quiddity_delete(self):
        err, res = self.sio.call('quiddity.delete', data=4)
        time.sleep(1)
        self.assertIsNone(err)
        self.assertTrue(res)
        event, deleted_data = self.rcvd_data.pop()
        self.sio.logger.debug(self.rcvd_data)
        self.assertIsInstance(deleted_data[0], int)
