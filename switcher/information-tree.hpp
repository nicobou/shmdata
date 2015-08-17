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
 * @file  information-tree.h
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
#include "./any.hpp"

namespace switcher {
namespace data {
class Tree {
 public:
  using ptr = std::shared_ptr<Tree>;  // shared
  using ptrc = const Tree *;  // const
  using rptr = Tree *;  // raw
  using child_type = std::pair<std::string, Tree::ptr>;
  using childs_t = std::list<child_type>;
  using OnNodeFunction = std::function<void(const std::string &name,
                                             Tree::ptrc tree,
                                             bool is_array_element)>;
  using GetNodeReturn = std::pair<Tree::childs_t, Tree::childs_t::iterator>;
  
  // factory
  static Tree::ptr make();
  template<typename ValueType> static Tree::ptr make(ValueType data) {
    std::shared_ptr<Tree> tree;  //can't use make_shared because ctor is private
    tree.reset(new Tree(data));
    tree->me_ = tree;
    return tree;
  }
  static Tree::ptr make(const char *data);  // Tree will store a std::string
  
  // escaping dots from a keys ("." internally replaced by "__DOT__")
  static std::string escape_dots(const std::string &str);
  static std::string unescape_dots(const std::string &str);

  //walk
  static void preorder_tree_walk(Tree::ptrc tree,
                                 Tree::OnNodeFunction on_visiting_node,
                                 Tree::OnNodeFunction on_node_visited);
  static Tree::ptrc get_subtree(Tree::ptrc tree, const std::string &path);
  
  //const methods
  bool is_leaf() const;
  bool is_array() const;
  bool has_data() const;
  const Any &read_data () const;
  bool branch_is_leaf(const std::string &path) const;
  bool branch_has_data(const std::string &path) const;
  const Any &branch_read_data (const std::string &path) const;
  
  // get child keys - returning a newly allocated list
  std::list<std::string> get_child_keys(const std::string &path) const;

  // get child key in place, use with std::insert_iterator
  template<typename Iter>
  bool copy_and_insert_child_keys(std::string path, Iter pos) const {
    std::unique_lock<std::mutex> lock(mutex_);
    auto found = get_node(path);
    if (!found.first.empty()) {
      std::transform(found.second->second->childrens_.begin(),
                     found.second->second->childrens_.end(),
                     pos,
                     [](const child_type &child) {
                       return child.first;
                     });
      return true;
    }
    return false;
  }
  
  // get leaf values in a newly allocated container
  std::list<std::string> copy_leaf_values(const std::string &path) const;

  // get/set:
  Any get_data() const;
  void set_data(const Any &data);
  void set_data(const char *data);
  void set_data(std::nullptr_t ptr);
  Any get_data(const std::string &path) const;
  bool set_data(const std::string &path, const Any &data);
  bool set_data(const std::string &path, const char *data);
  bool set_data(const std::string &path, std::nullptr_t ptr);
  // graft will create the path and graft the tree,
  // or remove old one and replace will the new tree
  bool graft(const std::string &path, Tree::ptr);
  // return empty tree if nothing can be pruned
  Tree::ptr prune(const std::string &path);
  // get but not remove
  Tree::ptr get(const std::string &path);
  // return false if the path does not exist
  // when a path is tagged as an array, keys might be discarded
  // by some serializers, such as JSON
  bool tag_as_array(const std::string &path, bool is_array);

 private:
  Any data_ {};
  bool is_array_ {false};
  mutable childs_t childrens_ {};
  mutable std::mutex mutex_ {};
  std::weak_ptr<Tree> me_ {};
  
  Tree() {}
  explicit Tree(const Any &data);

  childs_t::iterator get_child_iterator(const std::string &key) const;
  static bool graft_next(std::istringstream &path, Tree *tree,
                         Tree::ptr leaf);
  GetNodeReturn get_node(const std::string &path) const;
  bool get_next(std::istringstream &path,
                childs_t &parent_list_result,
                childs_t::iterator &result_iterator) const;
};

}  // namespace data
}  // namespace switcher
#endif
