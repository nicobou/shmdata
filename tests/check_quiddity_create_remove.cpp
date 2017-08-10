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
#include <iostream>
#include <string>
#include <vector>
#include "switcher/quiddity-basic-test.hpp"
#include "switcher/switcher.hpp"

int main() {
  bool success = true;
  switcher::Switcher::ptr manager = switcher::Switcher::make_switcher("test_manager");
  std::vector<std::string> classes = manager->get_classes();

  for (auto& it : classes) {
    if (!switcher::QuiddityBasicTest::test_create(manager, it)) success = false;
  }

  gst_deinit();
  if (success)
    return 0;
  else
    return 1;
}
