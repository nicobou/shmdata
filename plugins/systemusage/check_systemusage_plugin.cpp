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

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "switcher/quiddity-basic-test.hpp"
#include "switcher/quiddity-manager.hpp"

#ifdef HAVE_CONFIG_H
#include "../../config.h"
#endif

void quiddity_created_removed_cb(std::string /*subscriber_name */,
                                 std::string quiddity_name,
                                 std::string signal_name,
                                 std::vector<std::string> params,
                                 void* user_data) {
  // g_print("%s: %s\n", signal_name.c_str(), params[0].c_str());
  switcher::QuiddityManager* ctx = static_cast<switcher::QuiddityManager*>(user_data);
  std::cout << ctx->use_tree<MPtr(&switcher::InfoTree::serialize_json)>(quiddity_name, params[0])
            << std::endl;
}

int main() {
  bool success = true;

  {
    switcher::QuiddityManager::ptr manager =
        switcher::QuiddityManager::make_manager("test_manager");
#ifdef HAVE_CONFIG_H
    gchar* usr_plugin_dir = g_strdup_printf("./%s", LT_OBJDIR);
    manager->scan_directory_for_plugins(usr_plugin_dir);
    g_free(usr_plugin_dir);
#else
    return 1;
#endif
    if (!switcher::QuiddityBasicTest::test_full(manager, "systemusage")) success = false;
  }  // end of scope is releasing the manager
  if (success)
    return 0;
  else
    return 1;
}
