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

int
main ()
{
  using namespace switcher::data;
   {
     Tree::ptr tree = make_tree (std::string ("truc"));
     assert (tree->is_leaf());
     std::string data = tree->get_data ();
     assert (0 == data.compare ("truc"));
   }
    {
      Tree::ptr tree = make_tree ("test");
      assert (tree->is_leaf());
    }
   {
     Tree::ptr tree = make_tree (1.2f);
     float val = tree->get_data ();
     assert (1.2f == val);
   }
   {
     Tree::ptr tree = make_tree ();
     tree->add_child ("child_key", make_tree (std::string ("child_data")));
     assert (!tree->is_leaf());
     tree->remove_child ("child_key");
     assert (tree->is_leaf());
   }
   return 0;
}
