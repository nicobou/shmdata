/*
 * This file is part of switcher-myplugin.
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

#include <cassert>
#include <iostream>
#include <string>
#include <vector>
#include "switcher/property-container.hpp"
#include "switcher/quiddity-basic-test.hpp"
#include "switcher/quiddity-manager.hpp"

#ifdef HAVE_CONFIG_H
#include "../../config.h"
#endif

int main() {
  {
    using namespace switcher;
    QuiddityManager::ptr manager = QuiddityManager::make_manager("test_manager");
#ifdef HAVE_CONFIG_H
    gchar* usr_plugin_dir = g_strdup_printf("./%s", LT_OBJDIR);
    manager->scan_directory_for_plugins(usr_plugin_dir);
    g_free(usr_plugin_dir);
#else
    return 1;
#endif
    assert(manager->create("vrpnsrc", "src") == "src");
    assert(QuiddityBasicTest::test_full(manager, "vrpnsrc"));
    assert(manager->create("vrpnsink", "sink") == "sink");
    assert(QuiddityBasicTest::test_full(manager, "vrpnsink"));

  }  // end of scope is releasing the manager
  return 0;
}
