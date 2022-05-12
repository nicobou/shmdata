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


class InfotreeTestCase(unittest.TestCase):

    logger = logging.getLogger(__name__)
    logging.basicConfig(level=logging.DEBUG,
                        format='%(name)s: %(funcName)s: %(message)s')

    @classmethod
    def setUpClass(cls):
        # initialize switcher
        global sw
        sw = pyquid.Switcher('pyquid')

    def test_user_data_tree(self):
        quid = sw.create('emptyquid')
        utree = quid.get_user_tree()
        self.assertTrue(utree.empty())

    def test_json_default_serialization(self):
        quid = sw.create('emptyquid')
        utree = quid.get_user_tree()
        self.assertTrue(utree.graft('.my.int', 5))
        self.assertTrue(utree.graft('.my.float', 1.5))
        self.assertTrue(utree.graft('.my.bool', True))
        self.assertTrue(utree.graft('.my.string',
                                    "L'hydre-univers tordant son corps écaillé d'astres"))
        # retrieve values
        self.assertEqual(utree.get('.my.int'), 5)
        self.assertTrue(utree.get('.my.float'), 1.5)
        self.assertTrue(utree.get('.my.bool'), True)
        self.assertTrue(utree.get('.my.string'),
                        "L'hydre-univers tordant son corps écaillé d'astres")

        # some infotree nodes do not have a value
        self.assertIsNone(utree.get('.my'))

    def test_json_array_serialization(self):
        quid = sw.create('emptyquid')
        utree = quid.get_user_tree()
        self.assertTrue(utree.graft('.my.list', [1, 2, 3]))
        self.assertTrue(utree.graft('.my.tuple', (1, 2, 3)))
        self.assertTrue(utree.graft('.my.dict', {'a': 1, 'b': 2}))
        # legacy array serialization
        self.assertTrue(utree.graft('.my.old.array.1', 1))
        self.assertTrue(utree.graft('.my.old.array.2', 2))
        self.assertTrue(utree.graft('.my.old.array.3', 3))
        self.assertTrue(utree.tag_as_array('.my.old.array'))
        # copy a tree
        duptree = utree.copy()
        self.assertEqual(str(utree), str(duptree))
        # prune some parts
        duptree.prune('.my.old')
        self.assertNotEqual(str(utree), str(duptree))
        # copy parts of a tree
        duptree = utree.copy('.my.old')
        self.assertEqual(duptree.get('array.1'), 1)

    def test_infotree_init(self):
        # empty init
        tree_1 = pyquid.InfoTree()
        self.assertEqual('null', tree_1.json())
        self.assertEqual(str(tree_1), tree_1.json())
        # init from a json string
        tree_2 = pyquid.InfoTree('{ "one" : 1.0 }')
        self.assertEqual(tree_2.get('one'), 1.0)
        self.assertEqual(str(tree_2), tree_2.json())
        # init from a dictionnary
        tree_3 = pyquid.InfoTree({"two": 2.0})
        self.assertEqual(tree_3.get('two'), 2.0)
        self.assertEqual(str(tree_3), tree_3.json())

    def test_graft_tree(self):
        tree = pyquid.InfoTree({"b1": 1, "b2": 2})
        branch = pyquid.InfoTree('[1, 2, 3]')
        self.assertTrue(tree.graft('b3', branch))
        self.assertEqual(tree.get('b3.0'), 1)
        self.assertEqual(tree.get('b3.1'), 2)
