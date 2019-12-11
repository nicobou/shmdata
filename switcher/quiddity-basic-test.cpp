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

#include "./quiddity-basic-test.hpp"
#include <iostream>

namespace switcher {
bool test::full(Switcher::ptr manager,
                const std::string& quiddity_class_name,
                InfoTree::ptr config) {
  if (!tree(manager, quiddity_class_name, config)) return false;
  if (!create(manager, quiddity_class_name, config)) return false;
  if (!startable(manager, quiddity_class_name, config)) return false;
  if (!properties(manager, quiddity_class_name, config)) return false;
  return true;
}

bool test::create(Switcher::ptr manager,
                  const std::string& quiddity_class_name,
                  InfoTree::ptr config) {
  auto log = manager->get_logger();
  {
    auto res = manager->quids<MPtr(&switcher::quid::Container::create)>(
        quiddity_class_name, quiddity_class_name, config.get());
    if (!res) {
      log->warning("% cannot be created: %", quiddity_class_name, res.msg());
      return true;  // true because some quiddity may not be created because of a
      // missing resource
    }
    if (res.msg() != quiddity_class_name) {
      log->error("% was created with the wrong name", quiddity_class_name);
      return false;
    }
    auto res_rm = manager->quids<MPtr(&switcher::quid::Container::remove)>(
        manager->quids<MPtr(&switcher::quid::Container::get_id)>(quiddity_class_name));
    if (!res_rm) {
      log->error("error while removing quiddity %: %", quiddity_class_name, res.msg());
      return false;
    }
  }
  {  // testing with generated name
    auto res = manager->quids<MPtr(&switcher::quid::Container::create)>(
        quiddity_class_name, std::string(), config.get());
    if (!res) {
      log->warning("%  cannot be created: %", quiddity_class_name, res.msg());
      return true;  // true because some quiddity may not be created because of a
                    // missing resource
    }
    if (res.msg().empty()) {
      log->error("creation did not generate a name for class %", quiddity_class_name);
      return false;
    }
    auto res_rm = manager->quids<MPtr(&switcher::quid::Container::remove)>(
        manager->quids<MPtr(&switcher::quid::Container::get_id)>(res.msg()));
    if (!res_rm) {
      log->error("error while removing quiddity: %", res.msg());
      return false;
    }
  }
  return true;
}

bool test::startable(Switcher::ptr manager,
                     const std::string& quiddity_class_name,
                     InfoTree::ptr config) {
  auto log = manager->get_logger();
  auto qrox = manager->quids<MPtr(&switcher::quid::Container::create)>(
      quiddity_class_name, quiddity_class_name, config.get());
  if (!qrox) {
    log->warning("% cannot be created (startable not actualy tested)", quiddity_class_name);
    // return true because some quiddity may not be created because of a
    // missing resource
    return true;
  }
  auto started_id = qrox.get()->prop<MPtr(&PContainer::get_id)>("started");
  if (0 != started_id) {
    qrox.get()->prop<MPtr(&PContainer::set<bool>)>(started_id, true);
    qrox.get()->prop<MPtr(&PContainer::set<bool>)>(started_id, false);
    qrox.get()->prop<MPtr(&PContainer::set<bool>)>(started_id, true);
  }
  if (!manager->quids<MPtr(&switcher::quid::Container::remove)>(qrox.get_id())) {
    log->error("error while removing quiddity % (startable test)", quiddity_class_name);
    return false;
  }
  return true;
}

bool test::tree(Switcher::ptr manager,
                const std::string& quiddity_class_name,
                InfoTree::ptr config) {
  auto log = manager->get_logger();
  auto qrox = manager->quids<MPtr(&switcher::quid::Container::create)>(
      quiddity_class_name, std::string(), config.get());
  if (!qrox) {
    log->warning("% cannot be created (tree not actualy tested)");
    return true;
  }
  qrox.get()->tree<MPtr(&InfoTree::serialize_json)>(".");
  manager->quids<MPtr(&switcher::quid::Container::remove)>(qrox.get_id());
  return true;
}

bool test::properties(Switcher::ptr manager,
                      const std::string& quiddity_class_name,
                      InfoTree::ptr config) {
  auto log = manager->get_logger();
  auto qrox = manager->quids<MPtr(&switcher::quid::Container::create)>(
      quiddity_class_name, std::string(), config.get());
  if (!qrox) {
    log->warning("% cannot be created (properties not actualy tested)", quiddity_class_name);
    return true;
  }
  auto quid = qrox.get();
  auto properties = quid->tree<MPtr(&InfoTree::get_child_keys)>(".property.");
  for (auto& it : properties) {
    // do not test if the property has been disabled
    if (quid->tree<MPtr(&InfoTree::branch_read_data<bool>)>(std::string(".property.") + it +
                                                            ".enabled.")) {
      auto default_value = quid->prop<MPtr(&PContainer::get_str_str)>(it);
      if (!default_value.empty() && quid->tree<MPtr(&InfoTree::branch_read_data<bool>)>(
                                        std::string(".property.") + it + ".writable.")) {
        if (!quid->prop<MPtr(&PContainer::set_str_str)>(it, default_value)) {
          log->error("property % for quiddity of class % cannot be set with its default value",
                     it,
                     quiddity_class_name);
          return false;
        }
      }
    }
  }
  return true;
}

}  // namespace switcher
