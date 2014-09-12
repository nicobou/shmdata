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

#include "./information-tree.hpp"
#include <algorithm>
#include <regex>
#include <iostream>

namespace switcher {
namespace data {

Tree::ptr Tree::make() {
  std::shared_ptr<Tree> tree;  //can't use make_shared because ctor is private
  tree.reset(new Tree());
  tree->me_ = tree;
  return tree;
}

Tree::ptr Tree::make(const char *data) {
  return make (std::string(data));
}

void
Tree::preorder_tree_walk(Tree::ptrc tree,
                         Tree::OnNodeFunction on_visiting_node,
                         Tree::OnNodeFunction on_node_visited) {
  std::unique_lock<std::mutex> lock(tree->mutex_);
  if (!tree->childrens_.empty()) {
    for (auto &it : tree->childrens_) {
      on_visiting_node(it.first, it.second, tree->is_array_);
      preorder_tree_walk(it.second, on_visiting_node, on_node_visited);
      on_node_visited(it.first, it.second, tree->is_array_);
    }
  }
}

Tree::Tree(const Any &data):data_(data) {
}

bool Tree::is_leaf() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return childrens_.empty();
}

bool Tree::is_array() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return is_array_;
}

bool Tree::has_data() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return !data_.is_null();
}

Any Tree::get_data(){
  std::unique_lock<std::mutex> lock(mutex_);
  return data_;
}

const Any Tree::read_data() const{
  std::unique_lock<std::mutex> lock(mutex_);
  return data_;
}

void Tree::set_data(const Any &data) {
  std::unique_lock<std::mutex> lock(mutex_);
  data_ = data;
}

void Tree::set_data(const char *data) {
  std::unique_lock<std::mutex> lock(mutex_);
  data_ = std::string(data);
}

void Tree::set_data(std::nullptr_t ptr) {
  std::unique_lock<std::mutex> lock(mutex_);
  data_ = ptr;
}

bool Tree::is_leaf(const std::string &path) const {
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty())
    return found.second->second->childrens_.empty();
  return false;
}

bool Tree::has_data(const std::string &path) const {
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty())
    return found.second->second->data_.not_null();
  return false;
}

Any Tree::get_data(const std::string &path) {
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty())
    return found.second->second->data_;
  Any res;
  return res;
}

const Any Tree::read_data(const std::string &path) const {
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty())
    return found.second->second->data_;
  const Any res;
  return res;
}

bool Tree::set_data(const std::string &path, const Any &data) {
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty()) {
    found.second->second->data_ = data;
    return true;
  }
  return false;
}

bool Tree::set_data(const std::string &path, const char *data) {
  return set_data(path, std::string(data));
}

bool Tree::set_data(const std::string &path, std::nullptr_t ptr) {
  return set_data(path, Any(ptr));
}

Tree::childs_t::iterator
Tree::get_child_iterator(const std::string &key) const {
  return std::find_if(childrens_.begin(),
                      childrens_.end(),
                      [key] (const Tree::child_type &s) {
                        return (0 == s.first.compare(key));
                      });
}

Tree::ptr Tree::prune(const std::string &path) {
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty()) {
    Tree::ptr res = found.second->second;
    found.first.erase(found.second);
    return res;
  }
  Tree::ptr res;
  return res;
}

Tree::ptr Tree::get(const std::string &path) {
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty())
    return found.second->second;
  // not found
  Tree::ptr res;
  return res;
}

Tree::GetNodeReturn Tree::get_node(const std::string &path) const {
  std::istringstream iss(path);
  Tree::childs_t child_list;
  Tree::childs_t::iterator child_iterator;
  if (get_next(iss, child_list, child_iterator)) {
    // asking root node
    // std::cout << "asking root node" << std::endl;
  }
  return std::make_pair(child_list, child_iterator);
}

bool
Tree::get_next(std::istringstream &path,
               Tree::childs_t &parent_list_result,
               Tree::childs_t::iterator &iterator_result) const {
  std::string child_key;
  if (!std::getline(path, child_key, '.'))
    return true;
  if (child_key.empty()) {
    return this->get_next(path, parent_list_result, iterator_result);
  }

  auto it = get_child_iterator(child_key);
  if (childrens_.end() != it) {
    if (it->second->get_next(path, parent_list_result, iterator_result)) {
      iterator_result = it;
      parent_list_result = childrens_;
    }
    return false;
  }
  return false;
}

bool Tree::graft(const std::string &where, Tree::ptr tree) {
  if (!tree)
    return false;
  std::unique_lock<std::mutex> lock(mutex_);
  std::istringstream iss(where);
  return !graft_next(iss, this, tree);
}

bool
Tree::graft_next(std::istringstream &path,
                 Tree *tree,
                 Tree::ptr leaf) {
  std::string child;
  if (!std::getline(path, child, '.'))
    return true;
  if (child.empty())  // in case of two or more consecutive dots
    return graft_next(path, tree, leaf);
  auto it = tree->get_child_iterator(child);
  if (tree->childrens_.end() != it) {
    if (graft_next(path, it->second.get(), leaf))  // graft on already existing child
      it->second = leaf;  // replacing the previously empy tree with the one to graft
  } else {
    Tree::ptr child_node = make();
    tree->childrens_.emplace_back(child, child_node);
    if (graft_next(path, child_node.get(), leaf))   // graft on already existing child
    {
      // replacing empty tree for replacement by leaf
      tree->childrens_.pop_back();
      tree->childrens_.emplace_back(child, leaf);
    }
  }
  return false;
}

bool Tree::tag_as_array(const std::string &path, bool is_array) {
  Tree::ptr tree = Tree::get(path);
  if (!(bool) tree)
    return false;
  tree->is_array_ = is_array;
  return true;
}

bool Tree::is_array(const std::string &path) const {
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty())
    return found.second->second->is_array_;
  return false;
}

bool Tree::is_leaf(Tree::ptrc tree) {
  return tree->is_leaf();
}

bool Tree::is_array(Tree::ptrc tree) {
  return tree->is_array();
}

bool Tree::has_data(Tree::ptrc tree) {
  return tree->has_data();
}

const Any Tree::read_data (Tree::ptrc tree) {
  return tree->read_data();
}

bool Tree::is_leaf(Tree::ptrc tree, const std::string &path) {
  return tree->is_leaf(path);
}

bool Tree::is_array(Tree::ptrc tree, const std::string &path) {
  return tree->is_array(path);
}

bool Tree::has_data(Tree::ptrc tree, const std::string &path) {
  return tree->has_data(path);
}

const Any Tree::read_data (Tree::ptrc tree, const std::string &path) {
  return tree->read_data(path);
}
}  // namespace data
}  // namespace switcher
