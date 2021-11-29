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

#include "./container.hpp"

#include "../shmdata/caps/utils.hpp"
#include "../switcher.hpp"
#include "../utils/scope-exit.hpp"

namespace switcher {
namespace quiddity {
Container::ptr Container::make_container(Switcher* switcher, Factory* factory, log::Base* log) {
  Container::ptr container(new Container(switcher, factory, log));
  container->me_ = container;
  return container;
}

Container::Container(Switcher* switcher, Factory* factory, log::Base* log)
    : log::Logged(log), factory_(factory), switcher_(switcher) {}

Qrox Container::create(const std::string& quiddity_kind,
                       const std::string& nickname,
                       InfoTree::ptrc override_config) {
  // create a quiddity qrox
  auto res = quiet_create(quiddity_kind, nickname, override_config);
  // skip signals if creation failed
  if (!res) return res;

  // Copy the registered callbacks map
  // @NOTE: This is required in case a callback modifies it
  auto on_created_map = on_created_cbs_;
  // Iterate over callbacks
  for(auto& pair : on_created_map) {
    // Call second element of the pair which is the callback,
    // passing the current quiddity id as an argument.
    pair.second(res.get_id());
    if (on_created_cbs_.empty()) break;
  }
  // return the qrox
  return res;
}

Qrox Container::quiet_create(const std::string& quiddity_kind,
                             const std::string& raw_nickname,
                             InfoTree::ptrc override_config) {
  // checks before creation
  if (!switcher_->factory<MPtr(&Factory::exists)>(quiddity_kind)) {
    return Qrox(false, "unknown Quiddity kind");
  }

  // searching for a free id
  // note id is added into ids before creation because it is possibly used by quiddities
  // during initialization, if initilization fails, id will be removed searching for an available id
  auto cur_id = ids_.allocate_id();
  if (Ids::kInvalid == cur_id) {
    Qrox(false, "no more id available for Quiddity creation");
  }

  // nickname
  std::string nick;
  if (raw_nickname.empty()) {
    nick = quiddity_kind + std::to_string(cur_id);
  } else {
    nick = raw_nickname;
    for (const auto& it : quiddities_) {
      if (nick == it.second->get_nickname()) return Qrox(false, "nickname unavailable");
    }
  }

  // building configuration for quiddity creation
  InfoTree::ptr tree;
  auto conf = switcher_->conf<MPtr(&Configuration::get)>();
  if (conf) {
    tree = conf->get_tree(quiddity_kind);
    if (tree->empty()) tree = conf->get_tree("bundle." + quiddity_kind);
  }

  // creation
  Quiddity::ptr quiddity =
      factory_->create(quiddity_kind,
                       Config(cur_id,
                              nick,
                              quiddity_kind,
                              InfoTree::merge(tree.get(), override_config).get(),
                              this,
                              get_log_ptr()));
  if (!quiddity) {
    ids_.release_last_allocated_id();
    return Qrox(false, "Quiddity creation error");
  }

  if (!(*quiddity.get())) {
    ids_.release_last_allocated_id();
    return Qrox(false, "Quiddity initialization error");
  }
  quiddities_[cur_id] = quiddity;

  return Qrox(true, nick, cur_id, quiddity.get());
}

/**
* @brief Send confirmation signal for quiddity creation.
*        Use after a Container::quiet_create call.
*
* @param quid The quiddity instance returned by Container::quiet_create.
*/
void Container::notify_quiddity_created(Quiddity *quid) {
  // We work on a copy in case a callback modifies the map of registered callbacks
  auto cbs_map = on_created_cbs_;
  for (auto& pair : cbs_map) {
    auto callback = pair.second;
    callback(quid->get_id());
    if (on_created_cbs_.empty()) break;  // In case the map gets reset in the callback, e.g bundle
  }
};

BoolLog Container::remove(qid_t id) {
  // We work on a copy in case a callback modifies the map of registered callbacks
  const auto tmp_removed_cbs_ = on_removed_cbs_;
  for (const auto& cb : tmp_removed_cbs_) {
    cb.second(id);
    if (on_removed_cbs_.empty()) break;  // In case the map gets reset in the callback, e.g. bundle
  }
  auto res = quiet_remove(id);
  if (!res) return res;
  return res;
}

BoolLog Container::quiet_remove(qid_t id) {
  if (!ids_.release_id(id)) return BoolLog(false, "quiddity not found");
  quiddities_.erase(id);
  return BoolLog(true);
}

std::vector<std::string> Container::get_nicknames() const {
  std::vector<std::string> res;
  for (const auto& it : quiddities_) res.push_back(it.second->get_nickname());
  return res;
}

std::vector<qid_t> Container::get_ids() const { return ids_.get_ids(); }

InfoTree::ptr Container::get_quiddities_description() {
  auto tree = InfoTree::make();
  tree->graft("quiddities", InfoTree::make());
  tree->tag_as_array("quiddities", true);
  auto subtree = tree->get_tree("quiddities");
  for (const auto& it : quiddities_) {
    if (it.second) {
      auto quid = it.second;
      const auto id = std::to_string(quid->get_id());
      subtree->graft(id + ".id", InfoTree::make(quid->get_id()));
      subtree->graft(id + ".kind", InfoTree::make(quid->get_kind()));
    }
  }
  return tree;
}

InfoTree::ptr Container::get_quiddity_description(qid_t id) {
  auto it = quiddities_.find(id);
  if (quiddities_.end() == it) return InfoTree::make();
  auto tree = InfoTree::make();
  tree->graft(".id", InfoTree::make(it->second->get_id()));
  tree->graft(".kind", InfoTree::make(it->second->get_kind()));
  return tree;
}

Quiddity::ptr Container::get_quiddity(qid_t id) {
  auto it = quiddities_.find(id);
  if (quiddities_.end() == it) {
    debug("quiddity % not found, cannot provide ptr", std::to_string(id));
    Quiddity::ptr empty_quiddity_ptr;
    return empty_quiddity_ptr;
  }
  return it->second;
}

unsigned int Container::register_creation_cb(OnCreateRemoveCb cb) {
  static unsigned int id = 0;
  id %= std::numeric_limits<unsigned int>::max();
  me_.lock();
  on_created_cbs_[++id] = cb;
  return id;
}

unsigned int Container::register_removal_cb(OnCreateRemoveCb cb) {
  static unsigned int id = 0;
  id %= std::numeric_limits<unsigned int>::max();
  me_.lock();
  on_removed_cbs_[++id] = cb;
  return id;
}

void Container::unregister_creation_cb(unsigned int id) {
  me_.lock();
  auto it = on_created_cbs_.find(id);
  if (it != on_created_cbs_.end()) on_created_cbs_.erase(it);
}

void Container::unregister_removal_cb(unsigned int id) {
  me_.lock();
  auto it = on_removed_cbs_.find(id);
  if (it != on_removed_cbs_.end()) on_removed_cbs_.erase(it);
}

void Container::reset_create_remove_cb() {
  me_.lock();
  on_created_cbs_.clear();
  on_removed_cbs_.clear();
}

qid_t Container::get_id(const std::string& nickname) const {
  for (const auto& it : quiddities_) {
    if (nickname == it.second->get_nickname()) return it.first;
  }
  // not found
  return 0;
}

std::string Container::get_nickname(qid_t id) const {
  return quiddities_.find(id)->second->get_nickname();
}

Qrox Container::get_qrox(qid_t id) {
  return Qrox(true, get_nickname(id), id, get_quiddity(id).get());
}

}  // namespace quiddity
}  // namespace switcher

