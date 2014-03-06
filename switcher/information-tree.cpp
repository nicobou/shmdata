/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "information-tree.h"
#include <algorithm>

namespace switcher { 
  namespace data {
    
    Tree::ptr make_tree ()
    {
      return std::make_shared<Tree> ();
    } 

    template <typename ValueType>
      Tree::ptr make_tree (const ValueType &value);

    Tree::Tree () :
      data_ (),
      childrens_ ()
    {}

    Tree::~Tree ()
    {}

    Tree::Tree (const Any &data) :
      data_ (data),
      childrens_ ()
    {}

    bool
    Tree::is_leaf () const
    {
      return childrens_.empty ();
    }

    void 
    Tree::add_child (const std::string &key, Tree::ptr child)
    {
      childrens_.emplace_back (key, child);
    }
    
    Any
    Tree::get_data () const
    {
      return data_;
    }

    void
    Tree::set_data (const Any &data)
    {
      data_ = data;
    }
   
    void
    Tree::remove_child (const std::string &key)
    {
      auto it = std::find_if (childrens_.begin (), 
			      childrens_.end (),
			      [key] (const std::pair <std::string, Tree::ptr>& s) 
			      { return 0 == s.first.compare (key); }
			      );
      if (childrens_.end () != it)
	childrens_.erase (it);
    }
  } // end of namespace Information
}  // end of namespace switcher
