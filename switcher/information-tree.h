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
#include <algorithm>
#include <list>
#include <memory>
#include <type_traits>
#include <mutex>
#include "any.h"

namespace switcher { 
  namespace data {

    //-------- 
    class Tree
    {
    public:
      typedef std::shared_ptr<Tree> ptr;
      typedef std::pair <std::string, Tree::ptr> child_type;
      typedef std::list<child_type> child_list_type;
      Tree ();
      ~Tree ();
      Tree (const Any &data);
      bool is_leaf ();
      bool has_data ();
      Any get_data ();
      void set_data (const Any &data);
      void set_data (const char *data);
      void set_data (std::nullptr_t ptr);

      //path based methods
      bool is_leaf (const std::string &path);
      bool has_data (const std::string &path);
      Any get_data (const std::string &path);
      bool set_data (const std::string &path, const Any &data);
      bool set_data (const std::string &path, const char *data);
      bool set_data (const std::string &path, std::nullptr_t ptr);
      Tree::ptr get (const std::string &path);
      bool graft (const std::string &path, Tree::ptr);
      Tree::ptr prune (const std::string &path);
      
      //get child key in place, use with std::insert_iterator
      template <typename Iter>
	void
	get_child_keys (const std::string path, Iter pos)
	{
	  std::unique_lock <std::mutex> lock (mutex_);
	  auto found = get_node (path);
	  if (!found.first.empty ())
	    std::transform (found.second->second->childrens_.begin (),
			    found.second->second->childrens_.end (),
			    pos,
			    [] (const child_type &child) {return child.first;});
	}

      //get child keys - returning a newly allocated container
      template <template<class T, class = std::allocator<T> > class Container = std::list>
	Container<std::string>
	get_child_keys (const std::string path)
	{
	  Container<std::string> res;
	  std::unique_lock <std::mutex> lock (mutex_);
	  auto found = get_node (path);
	  if (!found.first.empty ())
	    {
	      res.resize (found.second->second->childrens_.size ());
	      std::transform (found.second->second->childrens_.begin (),
			      found.second->second->childrens_.end (),
			      res.begin (),
			      [] (const child_type &child) {return child.first;});
	    }
	  return res;
	}
      
    private:
      Any data_;
      child_list_type childrens_;
      std::mutex mutex_;
      child_list_type::iterator get_child_iterator (const std::string &key);
      static bool graft_next (std::istringstream &path, Tree *tree, Tree::ptr leaf);
      std::pair <Tree::child_list_type, Tree::child_list_type::iterator>
	get_node (const std::string &path);
      bool get_next (std::istringstream &path, 
		     child_list_type &parent_list_result, 
		     child_list_type::iterator &result_iterator);

      //walks
      template <typename T, typename OnVisitingNode, typename OnNodeVisited>
	friend void 
	preorder_tree_walk (Tree::ptr tree,
			    OnVisitingNode on_visiting_node,
			    OnNodeVisited on_node_visited,
			    T &user_data)
      {
	std::unique_lock <std::mutex> lock (tree->mutex_);
	if (!tree->childrens_.empty ())
	  {
	    for (auto &it : tree->childrens_)
	      {
		std::size_t size = it.second->childrens_.size ();
		on_visiting_node (it.first,  
				  it.second->get_data (),  
				  size, 
				  user_data); 
		preorder_tree_walk (it.second, 
				    on_visiting_node, 
				    on_node_visited,
				    user_data);
		on_node_visited (it.first,  
				 it.second->get_data (),  
				 size, 
				 user_data);
	      }
	  }
      }
    };

    //-------------- utils
    //constructor
    Tree::ptr make_tree (); 
    template <typename ValueType> 
      Tree::ptr make_tree (ValueType data) {return std::make_shared<Tree> (data);} 
    Tree::ptr make_tree (const char *data); //Tree will store a std::string

  } // end of "data" namespace 
}  // end of "switcher" namespace
#endif // ifndef
