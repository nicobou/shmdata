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
bool test::full(Switcher::ptr manager, const std::string& quiddity_class_name) {
  if (!tree(manager, quiddity_class_name)) return false;
  if (!create(manager, quiddity_class_name)) return false;
  if (!startable(manager, quiddity_class_name)) return false;
  if (!properties(manager, quiddity_class_name)) return false;
  return true;
}

bool test::create(Switcher::ptr manager, const std::string& quiddity_class_name) {
  {
    auto res = manager->quids<MPtr(&switcher::quid::Container::create)>(
        quiddity_class_name, quiddity_class_name, nullptr);
    if (!res) {
      std::cerr << quiddity_class_name << " cannot be created: " << res.msg() << '\n';
      return true;  // true because some quiddity may not be created because of a
      // missing resource
    }
    if (res.msg() != quiddity_class_name) {
      std::cerr << quiddity_class_name << " was created with the wrong name" << '\n';
      return false;
    }
    auto res_rm = manager->quids<MPtr(&switcher::quid::Container::remove)>(
        manager->quids<MPtr(&switcher::quid::Container::get_id)>(quiddity_class_name));
    if (!res_rm) {
      std::cerr << "error while removing quiddity: " << res.msg() << '\n';
      return false;
    }
  }
  {  // testing with generated name
    auto res = manager->quids<MPtr(&switcher::quid::Container::create)>(
        quiddity_class_name, std::string(), nullptr);
    if (!res) {
      std::cerr << quiddity_class_name << " cannot be created: " << res.msg() << '\n';
      return true;  // true because some quiddity may not be created because of a
                    // missing resource
    }
    if (res.msg().empty()) {
      std::cerr << "creation did not generate a name for class" << quiddity_class_name << '\n';
      return false;
    }
    auto res_rm = manager->quids<MPtr(&switcher::quid::Container::remove)>(
        manager->quids<MPtr(&switcher::quid::Container::get_id)>(res.msg()));
    if (!res_rm) {
      std::cerr << "error while removing quiddity: " << res.msg() << '\n';
      return false;
    }
  }
  return true;
}

bool test::startable(Switcher::ptr manager, const std::string& quiddity_class_name) {
  auto qrox = manager->quids<MPtr(&switcher::quid::Container::create)>(
      quiddity_class_name, quiddity_class_name, nullptr);
  if (!qrox) {
    std::cerr << quiddity_class_name << " cannot be created (startable not actualy tested)" << '\n';
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
    std::cerr << "error while removing quiddity " << quiddity_class_name << " (startable test)";
    return false;
  }
  return true;
}

bool test::tree(Switcher::ptr manager, const std::string& quiddity_class_name) {
  auto qrox = manager->quids<MPtr(&switcher::quid::Container::create)>(
      quiddity_class_name, std::string(), nullptr);
  if (!qrox) {
    std::cerr << quiddity_class_name << " cannot be created (tree not actualy tested)" << '\n';
    return true;
  }
  qrox.get()->tree<MPtr(&InfoTree::serialize_json)>(".");
  manager->quids<MPtr(&switcher::quid::Container::remove)>(qrox.get_id());
  return true;
}

bool test::properties(Switcher::ptr manager, const std::string& quiddity_class_name) {
  auto qrox = manager->quids<MPtr(&switcher::quid::Container::create)>(
      quiddity_class_name, std::string(), nullptr);
  if (!qrox) {
    std::cerr << quiddity_class_name << " cannot be created (properties not actualy tested)"
              << '\n';
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
          std::cerr << "property " << it << " for quiddity named " << qrox.msg()
                    << " class: " << quiddity_class_name << " cannot be set with its default value"
                    << '\n';
          return false;
        }
      }
    }
  }
  return true;
}

}  // namespace switcher
