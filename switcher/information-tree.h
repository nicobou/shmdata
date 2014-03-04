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


#ifndef __SWITCHER_INFORMATION_TREE_H__
#define __SWITCHER_INFORMATION_TREE_H__

#include <string>
#include <list>
#include <memory>


namespace switcher { 
  namespace information {
    class Tree
    {
    public:
      typedef std::shared_ptr<Tree> ptr;
      Tree ();
      ~Tree ();
      Tree (const std::string &data);
      std::string get_data () const;
      void set_data (const std::string &data);
      bool is_leaf () const;
      void add_child (const std::string &path, Tree::ptr child);
  
    private:
      std::string data_;
      std::list<std::pair <std::string, Tree::ptr>> childrens_;
    };

    Tree::ptr make_tree (const std::string &data); 
    Tree::ptr make_tree (); 

  } // end of namespace information
}  // end of namespace switcher
#endif // ifndef
