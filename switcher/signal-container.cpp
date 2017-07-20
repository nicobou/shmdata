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

#include "./signal-container.hpp"
#include "./property-internal-types.hpp"

namespace switcher {

SContainer::SContainer(InfoTree::ptr tree,
                       on_tree_grafted_cb_t on_tree_grafted_cb,
                       on_tree_pruned_cb_t on_tree_pruned_cb)
    : tree_(tree), on_tree_grafted_cb_(on_tree_grafted_cb), on_tree_pruned_cb_(on_tree_pruned_cb) {
  tree_->graft(".signal", InfoTree::make());
  tree_->tag_as_array(".signal", true);
}

bool SContainer::remove(sig_id_t sig_id) {
  auto it = strids_.find(sig_id);
  if (strids_.end() == it) return false;  // sig not found
  auto key = std::string("signal.") + it->second;
  tree_->prune(key);
  if (on_tree_pruned_cb_) on_tree_pruned_cb_(key);
  ids_.erase(it->second);
  strids_.erase(it);
  sigs_.erase(sig_id);
  return true;
}

SContainer::register_id_t SContainer::subscribe(sig_id_t id, notify_cb_t fun) const {
  auto sig = sigs_.find(id);
  if (sig == sigs_.end()) return 0;
  return sig->second->subscribe(std::forward<notify_cb_t>(fun));
}

SContainer::register_id_t SContainer::subscribe_by_name(const std::string& name,
                                                        notify_cb_t fun) const {
  return subscribe(get_id(name), std::forward<notify_cb_t>(fun));
}

bool SContainer::unsubscribe(sig_id_t id, register_id_t rid) const {
  auto sig = sigs_.find(id);
  if (sig == sigs_.end()) return false;
  return sig->second->unsubscribe(std::forward<register_id_t>(rid));
}

bool SContainer::unsubscribe_by_name(const std::string& name, register_id_t rid) const {
  return unsubscribe(get_id(name), rid);
}

SContainer::sig_id_t SContainer::get_id(const std::string& strid) const {
  const auto& it = ids_.find(strid);
  if (ids_.end() != it) return it->second;
  // accepting id converted to string
  auto id = prop::id_from_string(strid);
  if (sigs_.end() != sigs_.find(id)) return id;
  return 0;
}

std::vector<std::pair<std::string, SContainer::sig_id_t>> SContainer::get_ids() const {
  std::vector<std::pair<std::string, SContainer::sig_id_t>> ids;
  for (auto& sig : ids_) {
    ids.push_back(std::make_pair(sig.first, sig.second));
  }
  return ids;
}

std::string SContainer::get_name(sig_id_t id) const {
  const auto& it = strids_.find(id);
  if (strids_.end() != it) return it->second;
  return std::string();
}

std::map<SContainer::sig_id_t, std::string> SContainer::get_names() const { return strids_; }

SContainer::sig_id_t SContainer::make(const std::string& strid, const std::string& description) {
  sigs_[++counter_] = std::make_unique<Sig>();
  ids_[strid] = counter_;
  strids_[counter_] = strid;
  auto key = std::string("signal.") + strid;
  tree_->graft(key + ".name", InfoTree::make(strid));
  tree_->graft(key + ".description", InfoTree::make(description));
  if (on_tree_grafted_cb_) on_tree_grafted_cb_(key);
  return counter_;
}

void SContainer::notify(sig_id_t id, InfoTree::ptr&& tree) {
  auto sig = sigs_.find(id);
  if (sig == sigs_.end()) return;
  sig->second->notify(std::forward<InfoTree::ptr&&>(tree));
}

}  // namespace switcher
