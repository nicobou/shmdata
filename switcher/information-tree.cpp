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
#include <regex>
#include <iostream>

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
    Tree::set_data (Any &data)
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
			     std::cout << "key to find " << key << " key visited " << s.first << std::endl;
			     return (0 == s.first.compare (key));
			   });
    }
    
    Tree::ptr
    Tree::prune (const std::string &path)
    {
      auto found = get_node (path);
      if (!found.first.empty ())
       	{
	  Tree::ptr res = found.second->second;
	  found.first.erase (found.second);
	  return res;
	}
      Tree::ptr res = found.second->second;
      return res;
    }

    Tree::ptr 
    Tree::get  (const std::string &path)
    {
      auto found =  get_node (path);
      if (!found.first.empty ())
	return found.second->second;
      //no found
      std::cout << "NOT FPOUND" << std::endl;
      Tree::ptr res;
      return res;
    }

    std::pair <Tree::child_list_type, Tree::child_list_type::iterator>
    Tree::get_node (const std::string &path)
    {
      std::istringstream iss (path);
      Tree::child_list_type child_list;
      Tree::child_list_type::iterator child_iterator;
      get_next (iss, 
		child_list, 
		child_iterator);
      return std::make_pair (child_list, child_iterator);
    }
    

    bool 
    Tree::get_next (std::istringstream &path, 
		    Tree::child_list_type &parent_list_result, 
		    Tree::child_list_type::iterator &iterator_result)
    {
      std::string child_key;
      if (!std::getline (path, child_key, '.'))
	{
	  std::cout << "FOUND in get_next" << std::endl;
	  // iterator_result = this_it;
	  // parent_list_result = this_parent_list;
	  return true;
	}
      std::cout << "child_key: " << child_key << std::endl;
      if (child_key.empty ())
	{
	  std::cout << "child_key empty" << std::endl;
	  return this->get_next (path, 
				 parent_list_result, 
				 iterator_result);
	}
      
      auto it = get_child_iterator (child_key);
      std::cout << "blalbal" << std::endl;
      if (childrens_.end () != it)
	{
	  std::cout << "pliplipipl" << std::endl;
	  if( it->second->get_next (path, parent_list_result, iterator_result))
	    {
	      iterator_result = it;
	      parent_list_result = childrens_;
	    }
	  return false;
	}
      
      std::cout << "NOT FOUND IN get_next" << std::endl; 
      return false;
    }

    bool 
    Tree::graft (const std::string &where, Tree::ptr tree)
    {
      std::istringstream iss (where);
      return graft_next (iss, this, tree);
    }

    bool
    Tree::graft_next (std::istringstream &path, Tree *tree, Tree::ptr leaf)
    {
      std::string child;
      if (!std::getline (path, child, '.'))
	{
	  std::cout << "graft finished parsing" << std::endl;
	  return true;
	}
      if (child.empty ()) //in case of two or more consecutive dots
	  return graft_next (path, tree, leaf);
      std::cout << " child " << child << std::endl;
      auto it = tree->get_child_iterator (child);
      if (tree->childrens_.end () != it)
	{
	  if (graft_next (path, it->second.get (), leaf)) //graft on already existing child
	    {
	      std::cout << "replacing previous with" << std::endl;
	      it->second = leaf; // replacing the previously empy tree with the one to graft
	    }
  	}
      else
	{
	  Tree::ptr child_node = make_tree ();
	  tree->childrens_.emplace_back (child, child_node);
	  if (graft_next (path, child_node.get (), leaf)) //graft on already existing child
	    {
	      std::cout << "replacing previous with 2" << std::endl;
	      // replacing empty tree for replacement by leaf
	      tree->childrens_.pop_back ();
	      tree->childrens_.emplace_back (child, leaf);
	    }
	}
      return false;
    }
  } // end of namespace information
}  // end of namespace switcher
