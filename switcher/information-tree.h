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

  /**
   * @file   information-tree.h
   * 
   * @brief tree data structure for storing, formating and serializing informations  
   * 
   * The information tree is largely inspired from the boost' property tree. 
   * It provides a data structure that stores an arbitrarily deeply nested 
   * tree of values, indexed at each level by some key. Each node of the 
   * tree stores its own value, plus an ordered list of its subnodes and their keys. 
   * The tree allows easy access to any of its nodes by means of a path, 
   * which is a concatenation of multiple keys. 
   *
   */



#ifndef __SWITCHER_INFORMATION_TREE_H__
#define __SWITCHER_INFORMATION_TREE_H__

#include <string>
#include <list>
#include <memory>
#include "any.h"

namespace switcher { 
  namespace data {

    class Tree
    {
    public:
      typedef std::shared_ptr<Tree> ptr;
      typedef std::pair <std::string, Tree::ptr> child_type;
      typedef std::list<child_type> child_list_type;
      Tree ();
      ~Tree ();
      Tree (const Any &data);
      bool is_leaf () const;
      void graft (const std::string &key, Tree::ptr child);
      Tree::ptr prune (const std::string &key);
      Tree::ptr get (const std::string &key);
      
      Any get_data () const;
      void set_data (const Any &data);

    private:
      Any data_;
      child_list_type childrens_;
      child_list_type::iterator get_child_iterator (const std::string &key);
    };

    Tree::ptr make_tree (); 
    Tree::ptr make_tree (const char *data);

    template <typename ValueType>
    Tree::ptr make_tree (ValueType data)
    {
      return std::make_shared<Tree> (data);
    } 

  } // end of "data" namespace 
}  // end of "switcher" namespace
#endif // ifndef
