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
bool QuiddityBasicTest::test_full(Switcher::ptr manager, const std::string& quiddity_class_name) {
  if (!test_tree(manager, quiddity_class_name)) return false;
  if (!test_create(manager, quiddity_class_name)) return false;
  if (!test_description_by_class(manager, quiddity_class_name)) return false;
  if (!test_startable(manager, quiddity_class_name)) return false;
  if (!test_properties(manager, quiddity_class_name)) return false;
  if (!test_nickname(manager, quiddity_class_name)) return false;
  return true;
}

bool QuiddityBasicTest::test_create(Switcher::ptr manager, const std::string& quiddity_class_name) {
  // testing with a nick name
  std::string res_with_nick = manager->create(quiddity_class_name, quiddity_class_name);
  if (res_with_nick.compare(quiddity_class_name) != 0) {
    std::cerr << quiddity_class_name << " cannot be created (with nickname)" << '\n';
    return true;  // true because some quiddity may not be created because of a
                  // missing resource
  } else if (!manager->remove(res_with_nick)) {
    std::cerr << "error while removing quiddity " << quiddity_class_name << " (with nickname)"
              << '\n';
    return false;
  }
  // testing without nick name
  std::string res_without_nick = manager->create(quiddity_class_name);
  if (res_without_nick.empty()) {
    std::cerr << quiddity_class_name << " cannot be created (without nickname)" << '\n';
    return false;
  }
  if (!manager->remove(res_without_nick)) {
    std::cerr << "error while removing quiddity " << quiddity_class_name << " (without nickname)"
              << '\n';
    return false;
  }
  return true;
}

bool QuiddityBasicTest::test_startable(Switcher::ptr manager,
                                       const std::string& quiddity_class_name) {
  std::string name = manager->create(quiddity_class_name, quiddity_class_name);
  if (name.compare(quiddity_class_name) != 0) {
    std::cerr << quiddity_class_name << " cannot be created (startable not actualy tested)" << '\n';
    return true;  // true because some quiddity may not be created because of a
                  // missing resource
  }
  auto started_id = manager->use_prop<MPtr(&PContainer::get_id)>(name, "started");
  if (0 != started_id) {
    manager->use_prop<MPtr(&PContainer::set_str)>(name, started_id, "true");
    manager->use_prop<MPtr(&PContainer::set_str)>(name, started_id, "false");
    manager->use_prop<MPtr(&PContainer::set_str)>(name, started_id, "true");
  }
  if (!manager->remove(name)) {
    std::cerr << "error while removing quiddity " << quiddity_class_name << " (startable test)";
    return false;
  }
  return true;
}

bool QuiddityBasicTest::test_description_by_class(Switcher::ptr manager,
                                                  const std::string& quiddity_class_name) {
  // by class
  manager->get_methods_description_by_class(quiddity_class_name);
  return true;
}

bool QuiddityBasicTest::test_tree(Switcher::ptr manager, const std::string& quiddity_class_name) {
  std::string name = manager->create(quiddity_class_name);
  manager->use_tree<MPtr(&InfoTree::serialize_json)>(name, ".");
  manager->remove(name);
  return true;
}

bool QuiddityBasicTest::test_properties(Switcher::ptr manager,
                                        const std::string& quiddity_class_name) {
  std::string name = manager->create(quiddity_class_name);
  auto properties = manager->use_tree<MPtr(&InfoTree::get_child_keys)>(name, ".property.");
  for (auto& it : properties) {
    // do not test if the property has been disabled
    if (manager->use_tree<MPtr(&InfoTree::branch_read_data<bool>)>(
            name, std::string(".property.") + it + ".enabled.")) {
      auto default_value = manager->use_prop<MPtr(&PContainer::get_str_str)>(name, it);
      if (!default_value.empty() &&
          manager->use_tree<MPtr(&InfoTree::branch_read_data<bool>)>(
              name, std::string(".property.") + it + ".writable.")) {
        bool res = manager->use_prop<MPtr(&PContainer::set_str_str)>(name, it, default_value);
        if (!res) {
          std::cerr << "property " << it << " for quiddity named " << name
                    << " class: " << quiddity_class_name << " cannot be set with its default value"
                    << '\n';
          return false;
        }
      }
    }
  }
  return true;
}

bool QuiddityBasicTest::test_nickname(Switcher::ptr manager,
                                      const std::string& quiddity_class_name) {
  std::string name = manager->create(quiddity_class_name);
  if (name.empty()) return true;
  std::string nickname = name + " nickname";
  bool signal_received = false;

  if (name != manager->get_nickname(name)) {
    std::cerr << "nickname not initialised with name" << '\n';
    return false;
  }
  auto registration_id = manager->use_sig<MPtr(&switcher::SContainer::subscribe_by_name)>(
      name, "on-nicknamed", [&](const switcher::InfoTree::ptr& tree) {
        if (nickname == tree->get_value().as<std::string>()) signal_received = true;
      });
  if (0 == registration_id) return false;

  if (!manager->set_nickname(name, nickname)) {
    std::cerr << "cannot set nickname" << '\n';
    return false;
  }
  if (!manager->use_sig<MPtr(&switcher::SContainer::unsubscribe_by_name)>(
          name, "on-nicknamed", registration_id))
    return false;
  if (!signal_received) return false;
  if (nickname != manager->get_nickname(name)) {
    std::cerr << "issue getting the nickname" << '\n';
    return false;
  }
  if (manager->set_nickname("wrong name", nickname)) {
    std::cerr << "set_nickname with a wrong quiddity name must return false" << '\n';
    return false;
  }
  if (!manager->get_nickname("wrong name").empty()) {
    std::cerr << "get_nickname with a wrong quiddity name must return false" << '\n';
    return false;
  }
  return true;
}

}  // namespace switcher
