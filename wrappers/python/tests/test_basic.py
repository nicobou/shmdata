# This file is part of switcher python wrapper.
#
# libswitcher is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General
# Public License along with this library; if not, write to the
# Free Software Foundation, Inc., 59 Temple Place, Suite 330,
# Boston, MA 02111-1307, USA.

import logging
import pyquid
import unittest


class BasicTestCase(unittest.TestCase):

    logger = logging.getLogger(__name__)
    logging.basicConfig(level=logging.DEBUG,
                        format='%(name)s: %(funcName)s: %(message)s')

    @classmethod
    def setUpClass(cls):
        # initialize switcher
        global sw
        sw = pyquid.Switcher('pyquid')
        # create a window
        try:
            pyquid.Quiddity(switcher=sw, kind='glfwin', nickname='win1')
        except RuntimeError as e:
            cls.logger.exception(e)
            sw.create(kind='dummysink', nickname='win1')
        # create a video test source
        sw.create('videotestsrc', 'vid')

    @classmethod
    def tearDownClass(cls):
        # remove existing quiddities
        for quid in sw.quiddities:
            sw.remove(quid.id())

    def test_switcher_name(self):
        self.assertEqual('pyquid', sw.name())

    def test_switcher_version(self):
        self.assertTrue(sw.version())

    def test_quiddities(self):
        vid = sw.quiddities[-1]
        self.assertTrue(vid.nickname())
        self.assertTrue(vid.get_kind())
        nickname = 'vid1'
        vid.set_nickname(nickname)
        self.assertEqual(nickname, vid.nickname())

    def test_properties(self):
        vid = sw.quiddities[-1]
        # test properties
        self.assertTrue(vid.get_info_tree_as_json('.property'))
        propdoc = pyquid.InfoTree(vid.get_info_tree_as_json('.property'))
        self.assertFalse(propdoc.empty())
        self.assertTrue('started' in propdoc.get_key_values('id', False))
        self.assertFalse(vid.get('started'))

    def test_connect(self):
        win, vid = sw.quiddities[0], sw.quiddities[-1]
        # quiddities can be connected together using `shmdata`
        try:
            sfid = win.try_connect(vid)
        except RuntimeError as e:
            self.logger.exception(e)

        self.assertTrue(sfid.disconnect())
