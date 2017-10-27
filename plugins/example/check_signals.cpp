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
#include <vector>
#include "switcher/switcher.hpp"

unsigned int signal_counter = 0;

int main() {
  switcher::Switcher::ptr manager = switcher::Switcher::make_switcher("testing_signals");
  manager->scan_directory_for_plugins("./");

  // Test create/remove notification system
  unsigned int create_remove_counter = 0;
  manager->register_creation_cb([&](const std::string& /*quid_name*/) { ++create_remove_counter; });
  manager->register_removal_cb([&](const std::string& /*quid_name*/) { ++create_remove_counter; });
  assert(manager->create("signal", "signal-quiddity") == "signal-quiddity");

  auto registration_id = manager->use_sig<MPtr(&switcher::SContainer::subscribe_by_name)>(
      "signal-quiddity", "test-signal", [&](const switcher::InfoTree::ptr&) { ++signal_counter; });

  assert(0 != registration_id);

  manager->invoke("signal-quiddity", "emit-signal", nullptr, std::vector<std::string>());
  manager->invoke("signal-quiddity", "emit-signal", nullptr, std::vector<std::string>());
  manager->invoke("signal-quiddity", "emit-signal", nullptr, std::vector<std::string>());

  assert(manager->use_sig<MPtr(&switcher::SContainer::unsubscribe_by_name)>(
      "signal-quiddity", "test-signal", registration_id));
  // the following should not imply incrementation of signal_counter
  manager->invoke("signal-quiddity", "emit-signal", nullptr, std::vector<std::string>());

  assert(manager->remove("signal-quiddity"));

  gst_deinit();

  if (create_remove_counter == 2 && signal_counter == 3) return 0;

  return 1;
}
