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

#include "./method-container.hpp"

namespace switcher {

MContainer::MContainer(InfoTree::ptr tree,
                       on_tree_grafted_cb_t on_tree_grafted_cb,
                       on_tree_pruned_cb_t on_tree_pruned_cb)
    : tree_(tree), on_tree_grafted_cb_(on_tree_grafted_cb), on_tree_pruned_cb_(on_tree_pruned_cb) {
  tree_->graft(".method", InfoTree::make());
  tree_->tag_as_array(".method", true);
}

bool MContainer::remove(meth_id_t meth_id) {
  auto it = strids_.find(meth_id);
  if (strids_.end() == it) return false;  // meth not found
  auto key = std::string("method.") + it->second;
  tree_->prune(key);
  if (on_tree_pruned_cb_) on_tree_pruned_cb_(key);
  ids_.erase(it->second);
  strids_.erase(it);
  meths_.erase(meth_id);
  return true;
}

bool MContainer::enable(meth_id_t meth_id) {
  const auto& it = strids_.find(meth_id);
  if (strids_.end() == it) return false;
  auto key = std::string("method.") + it->second + ".enabled";
  tree_->graft(key, InfoTree::make(true));
  auto why_key = std::string("method.") + it->second + ".why_disabled";
  tree_->graft(why_key, InfoTree::make(""));
  if (on_tree_grafted_cb_) on_tree_grafted_cb_(key);
  return true;
}

bool MContainer::disable(meth_id_t meth_id, const std::string& why) {
  const auto& it = strids_.find(meth_id);
  if (strids_.end() == it) return false;
  auto key = std::string("method.") + it->second + ".enabled";
  tree_->graft(key, InfoTree::make(false));
  auto why_key = std::string("method.") + it->second + ".why_disabled";
  tree_->graft(why_key, InfoTree::make(why));
  if (on_tree_grafted_cb_) on_tree_grafted_cb_(key);
  return true;
}

MContainer::meth_id_t MContainer::get_id(const std::string& strid) const {
  const auto& it = ids_.find(strid);
  if (ids_.end() != it) return it->second;
  // accepting id converted to string
  auto id = MethodBase::id_from_string(strid);
  if (meths_.end() != meths_.find(id)) return id;
  return 0;
}

std::vector<std::pair<std::string, MContainer::meth_id_t>> MContainer::get_ids() const {
  std::vector<std::pair<std::string, MContainer::meth_id_t>> ids;
  for (auto& meth : ids_) {
    ids.push_back(std::make_pair(meth.first, meth.second));
  }
  return ids;
}

std::string MContainer::get_name(meth_id_t id) const {
  const auto& it = strids_.find(id);
  if (strids_.end() != it) return it->second;
  return std::string();
}

std::map<MContainer::meth_id_t, std::string> MContainer::get_names() const { return strids_; }

}  // namespace switcher
