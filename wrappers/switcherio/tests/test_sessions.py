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
import os

# test quiddity data
videotest_quid = {
    "kind": "videotestsrc",
    "nickname": "test",
    "properties": {},
    "user_data": {"testData": True}
}

signal_quid = {
    "kind": "signal",
    "nickname": "signal"
}


class SessionTestCase(SocketIOTestCase):
    session_name = 'session_test0.json'

    def tearDown(self):
        self.sio.call('session.clear')
        self.sio.call('session.remove', data=self.session_name)

    def create_quiddity(self, quid_model):
        data = tuple(quid_model.values())
        err, res = self.sio.call('quiddity.create', data=data)
        self.assertIsNone(err)
        self.assertEqual(res['kind'], videotest_quid['kind'])

    def check_quiddities(self, number_of_quids, quid_nickname=None):
        err, res = self.sio.call('switcher.quiddities')
        self.assertIsNone(err)
        self.assertEqual(len(res), number_of_quids)

        if quid_nickname:
            quid = [q for q in res if q['nickname'] == quid_nickname][0]
            self.assertIsNotNone(quid)

    def save_session(self, session_name):
        err, res = self.sio.call('session.save_as', data=session_name)
        self.assertIsNone(err)
        self.assertIsInstance(res, str)
        self.assertTrue(os.path.exists(res))
        return res

    def test_load_and_remove_a_session(self):
        self.create_quiddity(videotest_quid)
        self.check_quiddities(1)
        self.save_session(self.session_name)
        err, res = self.sio.call('session.reset')
        self.assertIsNone(err)
        self.check_quiddities(0)
        err, res = self.sio.call('session.load', data=self.session_name)
        self.assertIsNone(err)
        self.check_quiddities(1, videotest_quid['nickname'])
        err, res = self.sio.call('session.list')
        self.assertIsNone(err)
        self.assertEqual(self.session_name in res, True)
        err, res = self.sio.call('session.remove', data=self.session_name)
        self.assertIsNone(err)
        err, res = self.sio.call('session.list')
        self.assertIsNone(err)
        self.assertEqual(self.session_name in res, False)

    def test_reset_session(self):
        self.create_quiddity(videotest_quid)
        self.check_quiddities(1)
        err, res = self.sio.call('session.reset')
        self.assertIsNone(err)
        self.check_quiddities(0)
