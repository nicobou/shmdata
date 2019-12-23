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

#ifndef __SWITCHER_SIGNAL_CONTAINER_H__
#define __SWITCHER_SIGNAL_CONTAINER_H__

#include <map>
#include <memory>
#include <string>
#include "../../utils/counter-map.hpp"
#include "./sig.hpp"

namespace switcher {
class SContainer {
  friend class Bundle;  // replacing some methods like replace and delete
 public:
  using sig_id_t = Sig::sig_id_t;
  using notify_cb_t = Sig::notify_cb_t;
  using register_id_t = Sig::register_id_t;
  using on_tree_grafted_cb_t = std::function<void(const std::string& key)>;
  using on_tree_pruned_cb_t = std::function<void(const std::string& key)>;
  SContainer() = delete;
  // ctor will own tree and write into .property.
  SContainer(InfoTree::ptr tree,
             on_tree_grafted_cb_t on_tree_grafted_cb,
             on_tree_pruned_cb_t on_tree_pruned_cb);

  // ------------- use (const methods)
  // return 0 if id is not found
  sig_id_t get_id(const std::string& name) const;
  std::string get_name(sig_id_t id) const;
  std::vector<std::pair<std::string, SContainer::sig_id_t>> get_ids() const;
  std::map<sig_id_t, std::string> get_names() const;

  register_id_t subscribe(sig_id_t id, notify_cb_t fun) const;
  register_id_t subscribe_by_name(const std::string& name, notify_cb_t fun) const;
  bool unsubscribe(sig_id_t id, register_id_t rid) const;
  bool unsubscribe_by_name(const std::string& name, register_id_t rid) const;

  // remove
  bool remove(sig_id_t sig_id);
  sig_id_t make(const std::string& strid, const std::string& description);

  // notify
  void notify(sig_id_t sig_id, InfoTree::ptr&& ptr);

 private:
  sig_id_t counter_{0};
  std::map<sig_id_t, std::unique_ptr<Sig>> sigs_{};
  std::map<std::string, sig_id_t> ids_{};
  std::map<sig_id_t, std::string> strids_{};
  InfoTree::ptr tree_;
  on_tree_grafted_cb_t on_tree_grafted_cb_;
  on_tree_pruned_cb_t on_tree_pruned_cb_;
};

}  // namespace switcher
#endif
