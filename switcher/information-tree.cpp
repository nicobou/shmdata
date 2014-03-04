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
  namespace Information {
    
    Tree::ptr make_tree (const std::string &data)
    {
      return std::make_shared<Tree> (data);
    } 

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

  } // end of namespace Information
}  // end of namespace switcher
