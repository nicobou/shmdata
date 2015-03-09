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

#include "./quiddity-manager.hpp"
#include "./quiddity-basic-test.hpp"

namespace switcher {
bool
QuiddityBasicTest::test_full(QuiddityManager::ptr manager,
                             const std::string &quiddity_class_name) {
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  if (!test_get_info(manager, quiddity_class_name))
    return false;
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  if (!test_create(manager, quiddity_class_name))
    return false;
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  if (!test_description_by_class(manager, quiddity_class_name))
    return false;
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  if (!test_startable(manager, quiddity_class_name))
    return false;
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  return true;
}

bool
QuiddityBasicTest::test_create(QuiddityManager::ptr manager,
                               const std::string &quiddity_class_name) {
  // testing with a nick name
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  std::string res_with_nick =
      manager->create(quiddity_class_name, quiddity_class_name);
  if (res_with_nick.compare(quiddity_class_name) != 0) {
    g_warning("quiddity %s cannot be created (with nickname)",
              quiddity_class_name.c_str());
    return true;  // true because some quiddity may not be crated because of a missing resource
  } else if (!manager->remove(res_with_nick)) {
    g_warning("error while removing quiddity %s (with nickname)",
              quiddity_class_name.c_str());
    return false;
  }
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  // testing without nick name
  std::string res_without_nick = manager->create(quiddity_class_name);
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  if (res_without_nick.empty()) {
    g_warning("quiddity %s cannot be created (without nickname)",
              quiddity_class_name.c_str());
    return false;
  }
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  if (!manager->remove(res_without_nick)) {
    g_warning("error while removing quiddity %s (without nickname)",
              quiddity_class_name.c_str());
    return false;
  }
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  return true;
}

void on_started_cb(const std::string & /*subscriber_name */ ,
                   const std::string & /*quiddity_name */ ,
                   const std::string & /*property_name */ ,
                   const std::string & /*value */ ,
                   void * /*user_data */ ) {
  // g_print ("on_started_cb: %s, %s, %s, %s\n",
  //      subscriber_name.c_str (),
  //      quiddity_name.c_str (),
  //      property_name.c_str (),
  //      value.c_str ());
  return;
}

bool
QuiddityBasicTest::test_startable(QuiddityManager::ptr manager,
                                  const std::string &quiddity_class_name) {
  // g_print ("---- startable test for %s\n", quiddity_class_name.c_str ());
  std::string name =
      manager->create(quiddity_class_name, quiddity_class_name);
  if (name.compare(quiddity_class_name) != 0) {
    g_warning
        ("quiddity %s cannot be created (startable not actualy tested)",
         quiddity_class_name.c_str());
    return true;              // true because some quiddity may not be crated because of a missing resource
  }
  if (manager->has_property(name, "started")) {
    manager->make_property_subscriber("sub", on_started_cb, nullptr);
    manager->subscribe_property("sub", name, "started");
    // g_print ("has a started property\n");
    manager->set_property(name, "started", "true");
    // g_print ("started\n");
    manager->set_property(name, "started", "false");
    // g_print ("stoped\n");
    manager->set_property(name, "started", "true");
    // g_print ("restarted\n");
    manager->unsubscribe_property("sub", name, "started");
    manager->remove_property_subscriber("sub");
  }
  if (!manager->remove(name)) {
    g_warning("error while removing quiddity %s (startable test)",
              quiddity_class_name.c_str());
    return false;
  }
  return true;
}

bool
QuiddityBasicTest::test_description_by_class(QuiddityManager::ptr manager,
                                             const std::string &quiddity_class_name) {
  // by class
  manager->get_properties_description_by_class(quiddity_class_name);
  manager->get_methods_description_by_class(quiddity_class_name);
  manager->get_signals_description_by_class(quiddity_class_name);
  return true;
}

bool
QuiddityBasicTest::test_get_info(QuiddityManager::ptr manager,
                                 const std::string &quiddity_class_name) {
  std::string name = manager->create(quiddity_class_name);
  manager->get_info(name, ".");
  manager->remove(name);
  return true;
}

}  // namespace switcher
