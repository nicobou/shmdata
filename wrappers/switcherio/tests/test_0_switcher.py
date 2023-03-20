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


class SwitcherTestCase(SocketIOTestCase):
    """Test case for the `Switcher API`

    Available tests:
      - switcher.bundles
      - switcher.kinds
      - switcher.name
      - switcher.quiddities
      - switcher.version
    """

    def test_0_switcher_name(self):
        err, res = self.sio.call('switcher.name')
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, str)
        self.assertEqual(res, self.server_name)

    def test_1_switcher_version(self):
        err, res = self.sio.call('switcher.version')
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, str)
        self.assertEqual(res, self.version)

    def test_21_switcher_str_bundles(self):
        # describe a `bundle` configuration key for switcher
        bundles = '''{
            "bundle": {
                "testBundle": {
                    "pipeline": "property-quid name=Test",
                    "doc": {
                        "long_name": "Test",
                        "category": "test",
                        "tags": "writer",
                        "description": "Test"
                    }
                }
            }
        }'''
        # send the description to switcher
        err, res = self.sio.call('switcher.bundles', data=bundles)
        time.sleep(1)
        self.assertIsNone(err)
        self.assertTrue(res)

    def test_22_switcher_dict_bundles(self):
        # describe a `bundle` configuration key for switcher
        bundles = {
            "bundle": {
                "testBundle": {
                    "pipeline": "property-quid name=Test",
                    "doc": {
                        "long_name": "Test",
                        "category": "test",
                        "tags": "writer",
                        "description": "Test"
                    }
                }
            }
        }
        # send the description to switcher
        err, res = self.sio.call('switcher.bundles', data=bundles)
        time.sleep(1)
        self.assertIsNone(err)
        self.assertTrue(res)

    def test_3_switcher_kinds(self):
        err, res = self.sio.call('switcher.kinds')
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, dict)
        is_present = False
        for item in res['kinds']:
            if item['kind'] == 'testBundle':
                is_present = True
                break
        self.assertTrue(is_present, msg="The `testBundle` bundle is not in Switcher's kinds list")

    def test_4_switcher_quiddities(self):
        err, res = self.sio.call('switcher.quiddities')
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, list)

    def test_5_switcher_logging(self):
        err, res = self.sio.call("switcher.log", data=("debug", "Some important debugging text"))
        time.sleep(1)
        self.assertIsNone(err)
        self.assertTrue(res)
