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

#undef NDEBUG  // get assert in release mode

#include <gst/gst.h>
#include <cassert>
#include <iostream>
#include <vector>
#include "switcher/quiddity-manager.hpp"

unsigned int signal_counter = 0;

void cb_signal(const std::string& /*subscriber_name*/,
               const std::string& /*quiddity_name*/,
               const std::string& /*signal_name*/,
               const std::vector<std::string>& /*params*/,
               void* /*user_data*/) {
  ++signal_counter;
}

int main() {
  switcher::QuiddityManager::ptr manager =
      switcher::QuiddityManager::make_manager("testing_signals");

  gchar* usr_plugin_dir = g_strdup_printf("./");
  manager->scan_directory_for_plugins(usr_plugin_dir);
  g_free(usr_plugin_dir);

  assert(manager->make_signal_subscriber("signal_subscriber", cb_signal, nullptr));

  auto subscribers = manager->list_signal_subscribers();
  assert(subscribers.size() == 1);
  assert(subscribers[0] == "signal_subscriber");

  // Test create/remove notification system
  unsigned int create_remove_counter = 0;
  manager->register_creation_cb(
      [&create_remove_counter](const std::string& /*quid_name*/) { ++create_remove_counter; });
  manager->register_removal_cb(
      [&create_remove_counter](const std::string& /*quid_name*/) { ++create_remove_counter; });
  assert(manager->create("signal", "test_signal") == "test_signal");

  assert(manager->subscribe_signal("signal_subscriber", "test_signal", "test-signal"));

  manager->invoke("test_signal", "emit-signal", nullptr, std::vector<std::string>());
  manager->invoke("test_signal", "emit-signal", nullptr, std::vector<std::string>());
  manager->invoke("test_signal", "emit-signal", nullptr, std::vector<std::string>());

  assert(manager->list_subscribed_signals("signal_subscriber").size() == 1);
  assert(manager->remove("test_signal"));
  assert(manager->list_subscribed_signals("signal_subscriber").size() == 0);
  assert(manager->remove_signal_subscriber("signal_subscriber"));

  gst_deinit();

  if (create_remove_counter == 2 && signal_counter == 3) return 0;

  return 1;
}
