#!/usr/bin/env python3

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

import logging
import multiprocessing
import pyquid
import server
import socketio
import time
import unittest


class SocketIOTestCase(unittest.TestCase):
    # the socketio client instance
    sio = socketio.Client(logger=True)

    # websocket server default informations
    server_name = 'SocketIOServerTest'
    websocket_url = "ws://localhost:8000?version=Test"

    # list of event callbacks names
    events = []
    # data sent by the server will be eventually available
    # as soon as the signal callback gets triggered
    rcvd_data = []

    @classmethod
    def callback(cls, event, *data):
        cls.rcvd_data.append((event, *data))

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # register event callbacks for the socketio client
        self.sio.on('*', lambda ev, *d: self.__class__.callback(ev, d))
        # setup logging formatter
        formatter = logging.Formatter('%(name)s: %(message)s')
        self.sio.logger.handlers[0].setFormatter(formatter)

    @classmethod
    def setUpClass(cls):
        # init switcher
        cls.sw = pyquid.Switcher(cls.server_name, debug=True)
        cls.version = cls.sw.version()
        # set switcher instance for the server
        server.set_switcher_instance(cls.sw)
        # start websocket server in sub process
        cls._subproc = multiprocessing.Process(target=server.start_server)
        cls._subproc.start()
        time.sleep(3)
        # connect to websocket server
        cls.sio.connect(cls.websocket_url)

    @classmethod
    def tearDownClass(cls):
        # disconnect from the websocket server
        cls.sio.disconnect()
        # terminate the websocket server sub process
        cls._subproc.terminate()

    def tearDown(self):
        self.rcvd_data = []
