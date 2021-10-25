from .base import SocketIOTestCase
import json
import pyquid
import time


class SwitcherTestCase(SocketIOTestCase):
    """Test case for the `Switcher API`

    Available tests:
      - switcher.bundles
      - switcher.kinds.
      - switcher.name
      - switcher.quiddities
      - switcher.version
    """

    server_name = 'SocketIOServerTest'
    websocket_url = "ws://localhost:8000?version=Test"

    # no event callbacks registered for this test case yet
    events = []
    # data sent by the server will be eventually available
    # as soon as the signal callback gets triggered
    rcvd_data = None

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

    def test_2_switcher_bundles(self):
        # describe a `bundle` configuration key for switcher
        bundles = json.dumps({
            "bundle": {
                "testBundle": {
                    "pipeline": "dummy name=Test",
                    "doc": {
                        "long_name": "Test",
                        "category": "test",
                        "tags": "writer",
                        "description": "Test"
                    }
                }
            }
        })
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
        self.assertIsInstance(res, dict)
        self.assertEqual(res, {'quiddities': []})
