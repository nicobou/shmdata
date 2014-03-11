/*
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "switcher/information-tree.h"
#include <string>
#include <cassert>
#include <iostream>
int
main ()
{
  using namespace switcher::data;
  {//node data as std::string
    Tree::ptr tree = make_tree (std::string ("truc"));
    assert (tree->is_leaf ());
    std::string data = tree->get_data ();
    assert (0 == data.compare ("truc"));
  }
  {//node data as const char *
    Tree::ptr tree = make_tree ("test");
    assert (tree->is_leaf ());
  }
  {//node data as float
    Tree::ptr tree = make_tree (1.2f);
    float val = tree->get_data ();
    assert (1.2f == val);
  }
  {//graft a multiple childs and get them
    Tree::ptr tree = make_tree ();
    tree->graft ("...child1....child2..", make_tree (1.2f));
    assert (!tree->is_leaf ());
    Tree::ptr child2 = tree->get ("..child1.child2");
    assert (child2);
    assert (child2->is_leaf ());
    float data = child2->get_data ();
    assert (1.2f == data) ;
    Tree::ptr child1 = tree->get (".child1..");
    assert (!child1->is_leaf ());
    assert (!tree->get ("child1.foo"));//this is not a child
  }
  {//graft a childs and prune it
    Tree::ptr tree = make_tree ();
    tree->graft ("child1.child2", make_tree ());
    assert (!tree->is_leaf ());
    Tree::ptr child1 = tree->prune ("child1");
    assert (child1);
    assert (tree->is_leaf ());
    assert (!child1->is_leaf ());
    //child2 from the pruned child1
    Tree::ptr child2 = child1->prune ("child2");
    assert (child2);
    assert (child1->is_leaf ());
    assert (child2->is_leaf ());
  }
  {//is_leaf with path
    Tree::ptr tree = make_tree ();
    tree->graft ("child1.child2", make_tree ());
    assert (tree->is_leaf ("child1.child2"));
    assert (!tree->is_leaf ("child1"));
    assert (!tree->is_leaf ("foofoo"));
  }
  {//set/get data with path
    Tree::ptr tree = make_tree ();
    tree->graft ("child1.child2", make_tree ());
    //assert (tree->set_data ("child1.child2", "test"));
    //assert (tree->set_data ("child1", 1.2f));
  }
  return 0;
}
