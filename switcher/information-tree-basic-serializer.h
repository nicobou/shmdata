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
   * @file   information-tree-basic-serializer.h
   * 
   * @brief serialize and deserialize an information tree  
   * 
   * serializing as lines with absolute key path, space(s), value and new line
   * for instance:
   * .child1.child2.vol 0.6
   * .child1.child2.freq 440
   * .child1.child2.name myoscilator
   *
   */

#ifndef __SWITCHER_INFORMATION_TREE_BASIC_SERIALIZER_H__
#define __SWITCHER_INFORMATION_TREE_BASIC_SERIALIZER_H__

#include "information-tree.h"
#include <string>

namespace switcher { 
  namespace data {

    class BasicSerializer {
    public :
      static std::string serialize (Tree::ptr);
      static Tree::ptr deserialize (std::string &serialized);
    private:
      BasicSerializer () = delete;
      ~BasicSerializer () = delete;
      typedef struct {
	std::list<std::string> path_ {};
	std::string result_ {};
      } BasicSerializerData;
      static void on_visiting_node (std::string key, Any value, std::size_t n, BasicSerializerData &data);
      static void on_node_visited (std::string key, Any value, std::size_t n, BasicSerializerData &data);
      static std::string path_to_string (std::list<std::string> path);
    };

  } // end of "data" namespace 
}  // end of "switcher" namespace

#endif // ifndef
