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
 * @brief tree data structure for storing, formating and serializing
 * informations
 *
 * The information tree is largely inspired from the boost' property tree.
 * It provides a data structure that stores an arbitrarily deeply nested
 * tree of values, indexed at each level by some key. Each node of the
 * tree stores its own value, plus an ordered list of its subnodes and their
 * keys.
 * The tree allows easy access to any of its nodes by means of a path,
 * which is a concatenation of multiple keys.
 *
 */

#ifndef __SWITCHER_INFORMATION_TREE_H__
#define __SWITCHER_INFORMATION_TREE_H__

#include <algorithm>
#include <functional>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <vector>
#include "./any.hpp"

namespace switcher {

class InfoTree {
 public:
  using ptr = std::shared_ptr<InfoTree>;  // shared
  using ptrc = const InfoTree*;           // const
  using rptr = InfoTree*;                 // raw
  using child_type = std::pair<std::string, InfoTree::ptr>;
  using children_t = std::vector<child_type>;
  using OnNodeFunction =
      std::function<bool(const std::string& name, InfoTree::ptrc tree, bool is_array_element)>;
  using GetNodeReturn = std::pair<InfoTree::children_t*, InfoTree::children_t::size_type>;

  // factory
  static InfoTree::ptr make();
  static InfoTree::ptr copy(InfoTree::ptrc);
  // merge copy values from "second" tree into "first" tree
  static InfoTree::ptr merge(InfoTree::ptrc first, InfoTree::ptrc second);

  template <typename ValueType>
  static InfoTree::ptr make(ValueType data) {
    std::shared_ptr<InfoTree> tree;  // can't use make_shared because ctor is private
    tree.reset(new InfoTree(std::forward<ValueType>(data)));
    tree->me_ = tree;
    return tree;
  }
  // InfoTree will store a std::string
  static InfoTree::ptr make(const char* data);
  static InfoTree::ptr make_null();

  // collecting values (serialized)
  using collect_predicate_t = std::function<bool(const std::string& key, InfoTree::ptrc node)>;
  // the predicate can return false in order to stop searching in subtrees
  static std::list<Any> collect_values(InfoTree::ptrc tree,
                                       collect_predicate_t predicate,
                                       bool continue_search_in_siblings);

  // escaping dots from a keys ("." internally replaced by "__DOT__")
  static std::string escape_dots(const std::string& str);
  static std::string unescape_dots(const std::string& str);

  // walk
  static void preorder_tree_walk(InfoTree::ptrc tree,
                                 InfoTree::OnNodeFunction on_visiting_node,
                                 InfoTree::OnNodeFunction on_node_visited);
  // FIXME replace the following with a get_copy non static method
  static InfoTree::ptrc get_subtree(InfoTree::ptrc tree, const std::string& path);

  // const methods
  bool empty() const;
  bool is_leaf() const;
  bool is_array() const;
  bool has_data() const;
  const Any& read_data() const;
  bool branch_is_array(const std::string& path) const;
  bool branch_is_leaf(const std::string& path) const;
  bool branch_has_data(const std::string& path) const;
  template <typename T>
  T branch_read_data(const std::string& path) const {
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    auto found = get_node(path);
    if (nullptr != found.first) return (*found.first)[found.second].second->data_.copy_as<T>();
    return T();
  }

  // serialize
  std::string serialize_json(const std::string& path) const;

  // get child keys - returning a newly allocated list
  std::list<std::string> get_child_keys(const std::string& path) const;

  // get child key in place, use with std::insert_iterator
  template <typename Iter>
  bool copy_and_insert_child_keys(std::string path, Iter pos) const {
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    auto found = get_node(path);
    if (nullptr != found.first) {
      std::transform((*found.first)[found.second].second->children_.begin(),
                     (*found.first)[found.second].second->children_.end(),
                     pos,
                     [](const child_type& child) { return child.first; });
      return true;
    }
    return false;
  }

  // get leaf values in a newly allocated container
  std::list<std::string> copy_leaf_values(const std::string& path) const;

  // get/set:
  Any get_value() const;
  void set_value(const Any& data);
  void set_value(const char* data);
  void set_value(std::nullptr_t ptr);
  Any branch_get_value(const std::string& path) const;
  bool branch_set_value(const std::string& path, const Any& data);
  bool branch_set_value(const std::string& path, const char* data);
  bool branch_set_value(const std::string& path, std::nullptr_t ptr);

  // copy subtree
  InfoTree::ptr branch_get_copy(const std::string& path) const;

  // graft will create the path and graft the tree,
  // or remove old one and replace will the new tree
  bool graft(const std::string& path, InfoTree::ptr);
  // graft by value
  template <typename T>
  bool vgraft(const std::string& path, T val) {
    return graft(path, InfoTree::make(val));
  }
  // return empty tree if nothing can be pruned
  InfoTree::ptr prune(const std::string& path);
  // get but not remove
  InfoTree::ptr get_tree(const std::string& path);
  // return false if the path does not exist
  // when a path is tagged as an array, keys might be discarded
  // by some serializers, such as JSON
  bool tag_as_array(const std::string& path, bool is_array);
  bool make_array(bool is_array);

 private:
  Any data_{};
  bool is_array_{false};
  mutable children_t children_{};
  mutable std::recursive_mutex mutex_{};
  std::weak_ptr<InfoTree> me_{};

  InfoTree() {}
  template <typename U>
  InfoTree(const U& data) : data_(Any(data)) {}
  template <typename U>
  InfoTree(U&& data) : data_(Any(std::forward<U>(data))) {}
  explicit InfoTree(const Any& data);
  explicit InfoTree(Any&& data);
  std::pair<bool, children_t::size_type> get_child_index(const std::string& key) const;
  static bool graft_next(std::istringstream& path, InfoTree* tree, InfoTree::ptr leaf);
  GetNodeReturn get_node(const std::string& path) const;
  GetNodeReturn get_next(std::istringstream& path,
                         children_t* parent_vector_result,
                         children_t::size_type result_index) const;
  static bool path_is_root(const std::string& path);
};

}  // namespace switcher
#endif
