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

namespace switcher { 
  namespace data {
    
    Tree::ptr make_tree (const std::string &data)
    {
      return std::make_shared<Tree> (data);
    } 
    
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

    Tree::Tree (const std::string &data) :
      data_ (data),
      childrens_ ()
    {}

    std::string
    Tree::get_data () const
    {
      //FIXME
      return get_string<std::string> (&data_);
    }

    void 
    Tree::set_data (const std::string &data)
    {
      //data_ = Any (data);
    }

    bool
    Tree::is_leaf () const
    {
      return childrens_.empty ();
    }

    void 
    Tree::add_child (const std::string &path, Tree::ptr child)
    {
      //HERE handle the path
      childrens_.emplace_back (path, child);
    }
    
  } // end of namespace Information
}  // end of namespace switcher
