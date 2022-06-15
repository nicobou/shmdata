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


class CreationTestCase(unittest.TestCase):

    logger = logging.getLogger(__name__)
    logging.basicConfig(level=logging.DEBUG,
                        format='%(name)s: %(funcName)s: %(message)s')

    @classmethod
    def setUpClass(cls):
        global sw
        sw = pyquid.Switcher('creation')

    def test_list_kinds(self):
        self.assertTrue(len(sw.list_kinds()) > 0)

    def test_load_custom_kinds(self):
        description = '''{
            "bundle": {
                "testBundle" : {
                    "pipeline" : "dummy name=Test",
                    "doc" : {
                        "long_name" : "Test",
                        "category" : "test",
                        "tags" : "writer",
                        "description" : "Test"
                    }
                }
            }
        }'''
        sw.load_bundles(description)
        self.assertTrue("testBundle" in sw.list_kinds())

    def test_kinds_doc(self):
        self.assertTrue(len(sw.kinds_doc()) > 0)
        for name in sw.list_kinds():
            self.assertTrue(len(sw.kinds_doc()) > 0)

    def test_quiddities(self):
        audio = sw.create('audiotestsrc', 'aud1')
        self.assertTrue('aud1' in sw.list_quids())
        self.assertTrue(len(sw.quids_descr()) > 0)
        self.assertTrue(len(sw.quid_descr(audio.id())) > 0)
