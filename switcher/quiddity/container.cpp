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
                       const std::string& name,
                       InfoTree::ptrc override_config) {
  auto res = quiet_create(quiddity_class, name, override_config);
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
                             const std::string& raw_nick_name,
                             InfoTree::ptrc override_config) {
  // checks before creation
  if (!switcher_->factory<MPtr(&Factory::exists)>(quiddity_class)) {
    return Qrox(false, "unknown class");
  }
  std::string name;
  if (raw_nick_name.empty()) {
    name = quiddity_class + std::to_string(counters_.get_count(quiddity_class));
    while (names_.end() != names_.find(name))
      name = quiddity_class + std::to_string(counters_.get_count(quiddity_class));
  } else {
    name = Quiddity::string_to_quiddity_name(raw_nick_name);
  }
  if (names_.end() != names_.find(name)) {
    return Qrox(false, "name unavailable");
  }

  // building configuration for quiddity creation
  InfoTree::ptr tree;
  auto conf = switcher_->conf<MPtr(&Configuration::get)>();
  if (conf) {
    tree = conf->get_tree(quiddity_class);
    if (tree->empty()) tree = conf->get_tree("bundle." + quiddity_class);
  }

  // creation
  // note id is added into names before creation because it is possibly used by quiddities during
  // initialization, if initilization fails, id will be removed
  if (cur_id_ == std::numeric_limits<unsigned int>::max())
    cur_id_ = 1;
  else
    ++cur_id_;
  names_[name] = cur_id_;

  Quiddity::ptr quiddity =
      factory_->create(quiddity_class,
                       Config(name,
                              quiddity_class,
                              InfoTree::merge(tree.get(), override_config).get(),
                              this,
                              get_log_ptr()));
  if (!quiddity) {
    names_.erase(name);
    return Qrox(false, "abstract factory error");
  }

  if (!(*quiddity.get())) {
    debug("creation of % with name % failed", quiddity_class, quiddity->get_name());
    names_.erase(name);
    return Qrox(false, "quiddity initialization failed");
  }
  quiddities_[cur_id_] = quiddity;

  return Qrox(true, name, cur_id_, quiddity.get());
}

BoolLog Container::remove(qid_t id) {
  // We work on a copy in case a callback modifies the map of registered callbacks
  auto tmp_removed_cbs_ = on_removed_cbs_;
  for (auto& cb : tmp_removed_cbs_) {
    cb.second(id);
    if (on_removed_cbs_.empty()) break;  // In case the map gets reset in the callback, e.g bundle
  }
  auto res = quiet_remove(id);
  if (!res) return res;
  return res;
}

BoolLog Container::quiet_remove(qid_t id) {
  auto it = quiddities_.find(id);
  if (quiddities_.end() == it) {
    return BoolLog(false, "quiddity not found");
  }
  names_.erase(get_name(id));
  quiddities_.erase(id);
  return BoolLog(true);
}

std::vector<std::string> Container::get_names() const {
  std::vector<std::string> res;
  for (auto& it : names_) res.push_back(it.first);
  return res;
}

InfoTree::ptr Container::get_quiddities_description() {
  auto tree = InfoTree::make();
  tree->graft("quiddities", InfoTree::make());
  tree->tag_as_array("quiddities", true);
  auto subtree = tree->get_tree("quiddities");
  for (const auto& it : quiddities_) {
    if (it.second) {
      auto quid = it.second;
      std::string name = quid->get_name();
      subtree->graft(name + ".id", InfoTree::make(name));
      subtree->graft(name + ".class", InfoTree::make(quid->get_type()));
    }
  }
  return tree;
}

InfoTree::ptr Container::get_quiddity_description(qid_t id) {
  auto it = quiddities_.find(id);
  if (quiddities_.end() == it) return InfoTree::make();
  auto tree = InfoTree::make();
  tree->graft(".id", InfoTree::make(it->second->get_name()));
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

qid_t Container::get_id(const std::string& name) const {
  auto it = names_.find(name);
  if (names_.end() == it) return 0;
  return it->second;
}

std::string Container::get_name(qid_t id) const { return quiddities_.find(id)->second->get_name(); }

std::string Container::get_nickname(qid_t id) const {
  return quiddities_.find(id)->second->get_nickname();
}

std::string Container::get_name_from_caps(const std::string& caps) {
  std::string quiddity_name = "";
  const std::string manager = switcher::shmdata::caps::get_switcher_name(caps);
  if (manager.empty()) return "";
  if (manager != switcher_->get_name()) {
    // check if this manager is not actually a bundle from current switcher
    auto id = get_id(manager);
    if (id == 0) return "";
    quiddity_name = get_quiddity(id)->get_nickname();
  } else {
    const auto id = switcher::shmdata::caps::get_quiddity_id(caps);
    if (id != 0) quiddity_name = get_quiddity(id)->get_nickname();
  }
  return quiddity_name;
}

Qrox Container::get_qrox(qid_t id) { return Qrox(true, get_name(id), id, get_quiddity(id).get()); }

Qrox Container::get_qrox_from_name(const std::string& name) {
  auto id = get_id(name);
  if (0 == id) return Qrox(false, "not quiddity named " + name + " found");
  return get_qrox(id);
}

}  // namespace quiddity
}  // namespace switcher
