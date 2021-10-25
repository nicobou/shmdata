from .base import SocketIOTestCase
import json
import pyquid
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
      - quiddity.remove
    """

    server_name = 'SocketIOServerTest'
    websocket_url = "ws://localhost:8000?version=Test"

    # registering some generic event callbacks for testing purposes
    events = [
        'quiddity.created',
        'quiddity.removed',
        'nickname.updated'
    ]
    # data sent by the server will be eventually available
    # as soon as the signal callback gets triggered
    rcvd_data = None

    # test quiddity data
    test_quid = {
        "type": "dummy",
        "name": "test1",
        "nickname": "test_name",
        "props": {},
        "data": {"testData": True}
    }

    def test_01_quiddity_create(self):
        data = tuple(self.test_quid.values())
        err, res = self.sio.call('quiddity.create', data=data)
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, str)

    def test_02_property_set(self):
        err, res = self.sio.call('quiddity.create', data=('videotestsrc', 'vid1'))
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, str)
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
        self.assertIsInstance(self.rcvd_data[0], int)
        self.assertEqual(self.rcvd_data[1], "video1")

    def test_05_nickname_get(self):
        err, res = self.sio.call('nickname.get', data=2)
        time.sleep(1)
        self.assertIsNone(err)
        self.assertEqual(res, "video1")

    def test_06_quiddity_connect(self):
        err, res = self.sio.call('quiddity.create', data=('glfwin', 'win'))
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, str)
        err, res = self.sio.call('quiddity.connect', data=(3, 2))
        self.assertIsNone(err)
        self.assertTrue(res)

    def test_07_method_invokation(self):
        err, res = self.sio.call('quiddity.create', data=('methodquid', 'mquid'))
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, str)
        err, res = self.sio.call('method.invoke', data=(4, 'hello', ['Albert Camus']))
        time.sleep(1)
        self.assertIsNone(err)
        self.assertIsInstance(res, str)

    def test_08_quiddity_remove(self):
        err, res = self.sio.call('quiddity.remove', data='mquid')
        time.sleep(1)
        self.assertIsNone(err)
        self.assertTrue(res)
        self.assertIsInstance(self.rcvd_data[0], int)
