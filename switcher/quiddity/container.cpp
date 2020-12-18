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

#include <limits>

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

Qrox Container::create(const std::string& quiddity_class,
                       const std::string& nickname,
                       InfoTree::ptrc override_config) {
  auto res = quiet_create(quiddity_class, nickname, override_config);
  if (!res) return res;
  // We work on a copy in case a callback modifies the map of registered callbacks
  auto tmp_created_cbs = on_created_cbs_;
  for (auto& cb : tmp_created_cbs) {
    cb.second(cur_id_);
    if (on_created_cbs_.empty()) break;  // In case the map gets reset in the callback, e.g bundle
  }
  return res;
}

Qrox Container::quiet_create(const std::string& quiddity_class,
                             const std::string& raw_nickname,
                             InfoTree::ptrc override_config) {
  // checks before creation
  if (!switcher_->factory<MPtr(&Factory::exists)>(quiddity_class)) {
    return Qrox(false, "unknown class");
  }

  // searching for a free id
  // note id is added into ids before creation because it is possibly used by quiddities
  // during initialization, if initilization fails, id will be removed searching for an available id
  if (ids_.size() == std::numeric_limits<unsigned int>::max()) {
    Qrox(false, "no more id available for Quiddity creation");
  }
  do {
    if (cur_id_ == std::numeric_limits<unsigned int>::max())
      cur_id_ = 1;
    else
      ++cur_id_;
  } while (ids_.cend() != std::find(std::cbegin(ids_), std::cend(ids_), cur_id_));
  ids_.emplace_back(cur_id_);

  // nickname
  std::string nick;
  if (raw_nickname.empty()) {
    nick = quiddity_class + std::to_string(cur_id_);
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
    tree = conf->get_tree(quiddity_class);
    if (tree->empty()) tree = conf->get_tree("bundle." + quiddity_class);
  }

  // creation
  Quiddity::ptr quiddity =
      factory_->create(quiddity_class,
                       Config(cur_id_,
                              nick,
                              quiddity_class,
                              InfoTree::merge(tree.get(), override_config).get(),
                              this,
                              get_log_ptr()));
  if (!quiddity) {
    ids_.erase(ids_.end() - 1);
    return Qrox(false, "Quiddity creation error");
  }

  if (!(*quiddity.get())) {
    ids_.erase(ids_.end() - 1);
    return Qrox(false, "Quiddity initialization error");
  }
  quiddities_[cur_id_] = quiddity;

  return Qrox(true, nick, cur_id_, quiddity.get());
}

BoolLog Container::remove(qid_t id) {
  // We work on a copy in case a callback modifies the map of registered callbacks
  auto tmp_removed_cbs_ = on_removed_cbs_;
  for (auto& cb : tmp_removed_cbs_) {
    cb.second(id);
    if (on_removed_cbs_.empty()) break;  // In case the map gets reset in the callback, e.g. bundle
  }
  auto res = quiet_remove(id);
  if (!res) return res;
  return res;
}

BoolLog Container::quiet_remove(qid_t id) {
  auto it = std::find(ids_.begin(), ids_.end(), id);
  if (ids_.end() == it) {
    return BoolLog(false, "quiddity not found");
  }
  quiddities_.erase(id);
  ids_.erase(it);
  return BoolLog(true);
}

std::vector<std::string> Container::get_nicknames() const {
  std::vector<std::string> res;
  for (const auto& it : quiddities_) res.push_back(it.second->get_nickname());
  return res;
}

std::vector<qid_t> Container::get_ids() const { return ids_; }

InfoTree::ptr Container::get_quiddities_description() {
  auto tree = InfoTree::make();
  tree->graft("quiddities", InfoTree::make());
  tree->tag_as_array("quiddities", true);
  auto subtree = tree->get_tree("quiddities");
  for (const auto& it : quiddities_) {
    if (it.second) {
      auto quid = it.second;
      auto id = std::to_string(quid->get_id());
      subtree->graft(id + ".id", InfoTree::make(quid->get_id()));
      subtree->graft(id + ".class", InfoTree::make(quid->get_type()));
    }
  }
  return tree;
}

InfoTree::ptr Container::get_quiddity_description(qid_t id) {
  auto it = quiddities_.find(id);
  if (quiddities_.end() == it) return InfoTree::make();
  auto tree = InfoTree::make();
  tree->graft(".id", InfoTree::make(it->second->get_id()));
  tree->graft(".class", InfoTree::make(it->second->get_type()));
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

std::string Container::get_nickname_from_caps(const std::string& caps) {
  std::string nickname;

  const std::string manager = switcher::shmdata::caps::get_switcher_name(caps);
  if (manager.empty()) return "";
  if (manager != switcher_->get_name()) {
    // check if this manager is not actually a bundle from current switcher
    auto id = get_id(manager);
    if (id == 0) return "";
    nickname = get_quiddity(id)->get_nickname();
  } else {
    const auto id = switcher::shmdata::caps::get_quiddity_id(caps);
    if (id != 0) nickname = get_quiddity(id)->get_nickname();
  }
  return nickname;
}

Qrox Container::get_qrox(qid_t id) {
  return Qrox(true, get_nickname(id), id, get_quiddity(id).get());
}

Qrox Container::get_qrox_from_nickname(const std::string& nickname) {
  auto id = get_id(nickname);
  if (0 == id) return Qrox(false, "not quiddity named " + nickname + " found");
  return get_qrox(id);
}

}  // namespace quiddity
}  // namespace switcher
