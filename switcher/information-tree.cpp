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

#include <algorithm>
#include <regex>
#include <iostream>
#include "./information-tree.hpp"
#include "./information-tree-json.hpp"
#include "./string-utils.hpp"
#include "scope-exit.hpp"
namespace switcher {

InfoTree::ptr InfoTree::make() {
  std::shared_ptr<InfoTree> tree;  //can't use make_shared because ctor is private
  tree.reset(new InfoTree());
  tree->me_ = tree;
  return tree;
}

InfoTree::ptr InfoTree::make(const char *data) {
  return make (std::string(data));
}

void
InfoTree::preorder_tree_walk(InfoTree::ptrc tree,
                         InfoTree::OnNodeFunction on_visiting_node,
                         InfoTree::OnNodeFunction on_node_visited) {
  std::unique_lock<std::mutex> lock(tree->mutex_);
  if (!tree->childrens_.empty()) {
    for (auto &it : tree->childrens_) {
      on_visiting_node(it.first, it.second.get(), tree->is_array_);
      preorder_tree_walk(it.second.get(), on_visiting_node, on_node_visited);
      on_node_visited(it.first, it.second.get(), tree->is_array_);
    }
  }
}

InfoTree::InfoTree(const Any &data):data_(data) {
}

InfoTree::InfoTree(Any &&data):data_(data) {
}

bool InfoTree::is_leaf() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return childrens_.empty();
}

bool InfoTree::is_array() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return is_array_;
}

bool InfoTree::has_data() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return !data_.is_null();
}

Any InfoTree::get_data() const{
  std::unique_lock<std::mutex> lock(mutex_);
  return data_;
}

const Any &InfoTree::read_data() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return data_;
}

void InfoTree::set_data(const Any &data) {
  std::unique_lock<std::mutex> lock(mutex_);
  data_ = data;
}

void InfoTree::set_data(const char *data) {
  std::unique_lock<std::mutex> lock(mutex_);
  data_ = std::string(data);
}

void InfoTree::set_data(std::nullptr_t ptr) {
  std::unique_lock<std::mutex> lock(mutex_);
  data_ = ptr;
}

bool InfoTree::branch_is_leaf(const std::string &path) const {
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty())
    return found.second->second->childrens_.empty();
  return false;
}

bool InfoTree::branch_has_data(const std::string &path) const {
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty())
    return found.second->second->data_.not_null();
  return false;
}

Any InfoTree::get_data(const std::string &path) const {
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty())
    return found.second->second->data_;
  Any res;
  return res;
}

bool InfoTree::set_data(const std::string &path, const Any &data) {
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty()) {
    found.second->second->data_ = data;
    return true;
  }
  return false;
}

bool InfoTree::set_data(const std::string &path, const char *data) {
  return set_data(path, std::string(data));
}

bool InfoTree::set_data(const std::string &path, std::nullptr_t ptr) {
  return set_data(path, Any(ptr));
}

InfoTree::childs_t::iterator
InfoTree::get_child_iterator(const std::string &key) const {
  return std::find_if(childrens_.begin(),
                      childrens_.end(),
                      [key] (const InfoTree::child_type &s) {
                        return (0 == s.first.compare(key));
                      });
}

InfoTree::ptr InfoTree::prune(const std::string &path) {
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty()){
    On_scope_exit{found.first.erase(found.second);};
    return found.second->second;
  }
  return InfoTree::make();
}

InfoTree::ptr InfoTree::get(const std::string &path) {
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty())
    return found.second->second;
  // not found
  InfoTree::ptr res;
  return res;
}

InfoTree::GetNodeReturn InfoTree::get_node(const std::string &path) const {
  std::istringstream iss(path);
  InfoTree::childs_t child_list;
  InfoTree::childs_t::iterator child_iterator;
  if (get_next(iss, child_list, child_iterator)) {
    // asking root node
    child_list.push_back(std::make_pair("__ROOT__",
                                        me_.lock()));
    child_iterator = child_list.begin();
  }
  return std::make_pair(child_list, child_iterator);
}

bool InfoTree::get_next(std::istringstream &path,
                    InfoTree::childs_t &parent_list_result,
                    InfoTree::childs_t::iterator &iterator_result) const {
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

bool InfoTree::graft(const std::string &where, InfoTree::ptr tree) {
  if (!tree)
    return false;
  std::unique_lock<std::mutex> lock(mutex_);
  std::istringstream iss(where);
  return !graft_next(iss, this, tree);
}

bool
InfoTree::graft_next(std::istringstream &path,
                 InfoTree *tree,
                 InfoTree::ptr leaf) {
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
    InfoTree::ptr child_node = make();
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

bool InfoTree::tag_as_array(const std::string &path, bool is_array) {
  InfoTree::ptr tree = InfoTree::get(path);
  if (!(bool) tree)
    return false;
  tree->is_array_ = is_array;
  return true;
}

std::string InfoTree::escape_dots(const std::string &str) {
  return StringUtils::replace_char(str, '.', "__DOT__");
}

std::string InfoTree::unescape_dots(const std::string &str) {
  return StringUtils::replace_string(str, "__DOT__", ".");
}

std::list<std::string> InfoTree::get_child_keys(const std::string &path) const {
  std::list<std::string> res;
  std::unique_lock<std::mutex> lock(mutex_);
  auto found = get_node(path);
  if (!found.first.empty()) {
    res.resize(found.second->second->childrens_.size());
    std::transform(found.second->second->childrens_.cbegin(),
                   found.second->second->childrens_.cend(),
                   res.begin(),
                   [](const child_type &child) {
                     return child.first;
                   });
  }
  return res;
}

std::list<std::string> InfoTree::copy_leaf_values(const std::string &path) const {
  std::list<std::string> res;
  InfoTree::ptr tree;
  {  // finding the node
    std::unique_lock<std::mutex> lock(mutex_);
    auto found = get_node(path);
    if (found.first.empty()) 
      return res;
    tree = found.second->second;
  }
  preorder_tree_walk (tree.get(),
                      [&res](std::string /*key*/,
                             InfoTree::ptrc node,
                             bool /*is_array_element*/) {
                        if (node->is_leaf())
                          res.push_back(Any::to_string(node->read_data()));
                      },
                      [](std::string, InfoTree::ptrc, bool){});
  return res;
}

InfoTree::ptrc InfoTree::get_subtree(InfoTree::ptrc tree, const std::string &path) {
  std::unique_lock<std::mutex> lock(tree->mutex_);
  auto found = tree->get_node(path);
  return found.second->second.get();
}

std::string InfoTree::serialize_json(const std::string &path) const{
  std::unique_lock<std::mutex> lock(mutex_);

  auto found = get_node(path);
  std::cout << __FUNCTION__ << __LINE__ << path << "\n";
  return JSONSerializer::serialize(found.second->second.get());
}


}  // namespace switcher
