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
  {//graft a direct child and prune it
    Tree::ptr tree = make_tree ();
    tree->graft ("child_key", make_tree ());
    assert (!tree->is_leaf ());
    Tree::ptr child = tree->prune ("child_key");
    assert (tree->is_leaf ());
    assert (child->is_leaf ());
  }
  {//graft a direct child and get it
    Tree::ptr tree = make_tree ();
    tree->graft ("child_key", make_tree ());
    assert (!tree->is_leaf ());
    Tree::ptr child = tree->get ("child_key");
    assert (child);
    assert (!tree->is_leaf ());
    assert (child->is_leaf ());
  }
  {//graft a multiple childs and get them
    Tree::ptr tree = make_tree ();
    tree->graft ("...child1....child2..", make_tree (1.2f));
    std::cout << "--------------------" << std::endl;
    assert (!tree->is_leaf ());
    Tree::ptr child2 = tree->get ("..child1.child2");
    assert (child2);
    assert (child2->is_leaf ());
    float data = child2->get_data ();
    assert (1.2f == data) ;
    Tree::ptr child1 = tree->get (".child1..");
    assert (!child1->is_leaf ());
  }
  return 0;
}
