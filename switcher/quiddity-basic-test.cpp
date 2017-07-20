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

namespace switcher {
bool QuiddityBasicTest::test_full(QuiddityManager::ptr manager,
                                  const std::string& quiddity_class_name) {
  if (!test_tree(manager, quiddity_class_name)) return false;
  if (!test_create(manager, quiddity_class_name)) return false;
  if (!test_description_by_class(manager, quiddity_class_name)) return false;
  if (!test_startable(manager, quiddity_class_name)) return false;
  if (!test_properties(manager, quiddity_class_name)) return false;
  if (!test_nickname(manager, quiddity_class_name)) return false;
  return true;
}

bool QuiddityBasicTest::test_create(QuiddityManager::ptr manager,
                                    const std::string& quiddity_class_name) {
  // testing with a nick name
  std::string res_with_nick = manager->create(quiddity_class_name, quiddity_class_name);
  if (res_with_nick.compare(quiddity_class_name) != 0) {
    g_warning("quiddity %s cannot be created (with nickname)", quiddity_class_name.c_str());
    return true;  // true because some quiddity may not be created because of a
                  // missing resource
  } else if (!manager->remove(res_with_nick)) {
    g_warning("error while removing quiddity %s (with nickname)", quiddity_class_name.c_str());
    return false;
  }
  // testing without nick name
  std::string res_without_nick = manager->create(quiddity_class_name);
  if (res_without_nick.empty()) {
    g_warning("quiddity %s cannot be created (without nickname)", quiddity_class_name.c_str());
    return false;
  }
  if (!manager->remove(res_without_nick)) {
    g_warning("error while removing quiddity %s (without nickname)", quiddity_class_name.c_str());
    return false;
  }
  return true;
}

bool QuiddityBasicTest::test_startable(QuiddityManager::ptr manager,
                                       const std::string& quiddity_class_name) {
  std::string name = manager->create(quiddity_class_name, quiddity_class_name);
  if (name.compare(quiddity_class_name) != 0) {
    g_warning("quiddity %s cannot be created (startable not actualy tested)",
              quiddity_class_name.c_str());
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
    g_warning("error while removing quiddity %s (startable test)", quiddity_class_name.c_str());
    return false;
  }
  return true;
}

bool QuiddityBasicTest::test_description_by_class(QuiddityManager::ptr manager,
                                                  const std::string& quiddity_class_name) {
  // by class
  manager->get_methods_description_by_class(quiddity_class_name);
  return true;
}

bool QuiddityBasicTest::test_tree(QuiddityManager::ptr manager,
                                  const std::string& quiddity_class_name) {
  std::string name = manager->create(quiddity_class_name);
  manager->use_tree<MPtr(&InfoTree::serialize_json)>(name, ".");
  manager->remove(name);
  return true;
}

bool QuiddityBasicTest::test_properties(QuiddityManager::ptr manager,
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
          g_warning(
              "property %s for quiddity named %s (class %s)"
              " cannot be set with its default value",
              it.c_str(),
              name.c_str(),
              quiddity_class_name.c_str());
          return false;
        }
      }
    }
  }
  return true;
}

bool QuiddityBasicTest::test_nickname(QuiddityManager::ptr manager,
                                      const std::string& quiddity_class_name) {
  std::string name = manager->create(quiddity_class_name);
  if (name.empty()) return true;
  if (name != manager->get_nickname(name)) {
    g_warning("nickname not initialised with name");
    return false;
  }
  std::string nickname = name + " nickname";
  if (!manager->set_nickname(name, nickname)) {
    g_warning("cannot set nickname");
    return false;
  }
  if (nickname != manager->get_nickname(name)) {
    g_warning("issue getting the nickname");
    return false;
  }
  if (manager->set_nickname("wrong name", nickname)) {
    g_warning("set_nickname with a wrong quiddity name must return false");
    return false;
  }
  if (!manager->get_nickname("wrong name").empty()) {
    g_warning("get_nickname with a wrong quiddity name must return false");
    return false;
  }
  return true;
}

}  // namespace switcher
