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
    
    Tree::ptr 
    make_tree ()
    {
      return std::make_shared<Tree> ();
    } 

    Tree::ptr 
    make_tree (const char *data)
    {
      return std::make_shared<Tree> (std::string (data));
    } 

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
   
    Tree::child_list_type::iterator
    Tree::get_child_iterator (const std::string &key)
    {
      return std::find_if (childrens_.begin (), 
			   childrens_.end (),
			   [key] (const Tree::child_type& s) 
			   { 
			     return (0 == s.first.compare (key));
			   });
    }
    
    Tree::ptr
    Tree::prune (const std::string &key)
    {
      Tree::ptr res;
      auto it = get_child_iterator (key);
      if (childrens_.end () != it)
	{
	  res = it->second;
	  childrens_.erase (it);
	}
      return res;
    }

    Tree::ptr
    Tree::get (const std::string &key)
    {
      auto it = get_child_iterator (key);
      if (childrens_.end () != it)
	return (it->second);
      Tree::ptr res;
      return res;
    }

    void 
    Tree::graft (const std::string &key, Tree::ptr child)
    {
      childrens_.emplace_back (key, child);
    }
        
  } // end of namespace information
}  // end of namespace switcher
