/*
 * This file is part of switcher-top.
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

#include <chrono>
#include <thread>
#include <vector>
#include "switcher/quiddity-basic-test.hpp"

void quiddity_created_removed_cb(std::string /*subscriber_name */,
                                 std::string quiddity_name,
                                 std::string /*signal_name*/,
                                 std::vector<std::string> params,
                                 void* user_data) {
  switcher::Switcher* ctx = static_cast<switcher::Switcher*>(user_data);
  std::cout << ctx->use_tree<MPtr(&switcher::InfoTree::serialize_json)>(quiddity_name, params[0])
            << std::endl;
}

int main() {
  bool success = true;

  {
    switcher::Switcher::ptr manager = switcher::Switcher::make_switcher("test_manager");
    manager->scan_directory_for_plugins("./");

    if (!switcher::QuiddityBasicTest::test_full(manager, "systemusage")) success = false;
  }  // end of scope is releasing the manager
  if (success)
    return 0;
  else
    return 1;
}
