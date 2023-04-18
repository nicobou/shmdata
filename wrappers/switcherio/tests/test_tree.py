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
    "nickname": "test",
    "properties": {},
    "user_data": {"testData": True}
}

signal_quid = {
    "kind": "signal-quid",
    "nickname": "signal"
}


class TreeTestCase(SocketIOTestCase):
    # reverences on the created quiddities
    videotest_id = None
    signal_id = None

    def setUp(self):
        # creation of a videotestsrc quiddity
        data = tuple(videotest_quid.values())
        err, res = self.sio.call('quiddity.create', data=data)
        self.assertIsNone(err)
        self.videotest_id = res['id']
        # creation of a signal quiddity
        data = tuple(signal_quid.values())
        err, res = self.sio.call('quiddity.create', data=data)
        self.assertIsNone(err)
        self.signal_id = res['id']

    def tearDown(self):
        err, res = self.sio.call('session.clear')
        self.videotest_id = None
        self.signal_id = None
        self.rcvd_data = []


class UserTreeTestCase(TreeTestCase):
    """Test case for the `UserTree API`

      Available tests:
        - user_tree.get
        - user_tree.graft
        - user_tree.prune
        - on_user_tree.grafted
        - on_user_tree.pruned
      """

    events = [
        'user_tree.grafted',
        'user_tree.pruned'
    ]

    def test_create_quid_with_data(self):
        err, res = self.sio.call('user_tree.get', data=(self.videotest_id))
        self.assertEqual(res['testData'], True)

    def test_set_user_tree(self):
        err, res = self.sio.call('user_tree.graft', data=(self.videotest_id, 'testData', False))
        self.assertIsNone(err)
        self.assertIsInstance(res, dict)
        self.assertEqual(res['testData'], False)

    def test_get_user_tree(self):
        # add some data in the user tree
        self.sio.call('user_tree.graft', data=(self.videotest_id, 'testData', True))
        self.sio.call('user_tree.graft', data=(self.videotest_id, 'testTree.value', 4))

        # get a non empty user tree
        err, res = self.sio.call('user_tree.get', data=(self.videotest_id))
        self.assertIsNone(err)
        self.assertIsInstance(res, dict)
        self.assertEqual(res['testData'], True)

        # query a non empty value
        err, res = self.sio.call('user_tree.get', data=(self.videotest_id, 'testTree.value'))
        self.assertIsNone(err)
        self.assertIsInstance(res, int)
        self.assertEqual(res, 4)

        # query an empty value
        err, res = self.sio.call('user_tree.get', data=(self.videotest_id, 'testTree.fail'))
        self.assertIsNone(err)
        self.assertEqual(res, None)

    def test_user_tree_grafted(self):
        err, res = self.sio.call('user_tree.graft', data=(self.videotest_id, 'testData', False))
        time.sleep(1)
        event, grafted_data = self.rcvd_data.pop()
        self.assertEqual(event, 'user_tree.grafted')
        id, path, value = grafted_data
        self.assertEqual(id, self.videotest_id)
        self.assertEqual(path, 'testData')
        self.assertEqual(value, False)

    def test_user_tree_grafted_on_creation(self):
        videotest_quid2 = dict(videotest_quid)
        videotest_quid2['nickname'] = 'test2'
        err, res = self.sio.call('quiddity.create', data=tuple(videotest_quid2.values()))
        self.assertIsNone(err)
        time.sleep(1)
        event, grafted_data = self.rcvd_data.pop()
        self.assertEqual(event, 'user_tree.grafted')
        self.assertEqual(grafted_data, (res['id'], '.', None))

    def test_user_tree_pruned(self):
        self.sio.call('user_tree.graft', data=(self.videotest_id, 'testData', False))
        err, res = self.sio.call('user_tree.prune', data=(self.videotest_id, 'testData'))
        self.assertEqual(res, True)
        time.sleep(1)
        event, pruned_data = self.rcvd_data.pop()
        self.assertEqual(event, 'user_tree.pruned')
        id, path = pruned_data
        self.assertEqual(id, self.videotest_id)
        self.assertEqual(path, 'testData')
        err, res = self.sio.call('user_tree.prune', data=(self.videotest_id, 'testData'))
        self.assertEqual(res, False)


class ConnectionSpecsTestCase(TreeTestCase):
    """Test case for the `ConnectionSpecs API`

      Available tests:
        - connection_specs.get
      """

    def test_get_connection_specs(self):
        err, res = self.sio.call('connection_specs.get', data=(self.videotest_id))
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, dict)
        self.assertEqual(res['writer'][0]['can_do'][0], 'video/x-raw')


class InfoTreeTestCase(TreeTestCase):
    """Test case for the `InfoTree API`

      Available tests:
        - info_tree.get
        - info_tree.grafted
        - info_tree.pruned
      """

    events = [
        'info_tree.grafted',
        'info_tree.pruned'
    ]

    def test_get_info_tree(self):
        # get a non empty user tree
        err, res = self.sio.call('info_tree.get', data=(self.videotest_id))
        self.assertIsNone(err)
        self.assertIsInstance(res, dict)
        self.assertIsInstance(res['property'], list)

        # query a non empty value
        err, res = self.sio.call('info_tree.get', data=(self.videotest_id, 'property'))
        self.assertIsNone(err)
        self.assertIsInstance(res, list)

        # query an empty value
        err, res = self.sio.call('info_tree.get', data=(self.videotest_id, 'fail'))
        self.assertIsNone(err)
        self.assertEqual(res, None)

    def test_info_tree_grafted(self):
        err, res = self.sio.call('quiddity.invoke', data=(
            self.signal_id, 'do-graft-tree', []))
        time.sleep(1)
        event, grafted_data = self.rcvd_data.pop()  # we should get shmdata stats
        self.assertEqual(event, 'info_tree.grafted')
        self.assertIsNotNone(grafted_data)

    def test_info_tree_pruned(self):
        self.sio.call('quiddity.invoke', data=(
            self.signal_id, 'do-graft-tree', []))
        err, res = self.sio.call('quiddity.invoke', data=(
            self.signal_id, 'do-prune-tree', []))
        time.sleep(1)
        event, pruned_data = self.rcvd_data.pop()
        self.assertEqual(event, 'info_tree.pruned')
        self.assertIsNotNone(pruned_data)
