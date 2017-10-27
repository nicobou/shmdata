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

#undef NDEBUG  // get assert in release mode

#include <cassert>
#include "switcher/quiddity-basic-test.hpp"

int main() {
  {
    switcher::Switcher::ptr manager = switcher::Switcher::make_switcher("test_manager");

    manager->scan_directory_for_plugins("./");

    assert(switcher::QuiddityBasicTest::test_full(manager, "OSCctl"));
    assert(switcher::QuiddityBasicTest::test_full(manager, "OSCsink"));
  }  // end of scope is releasing the manager
  return 0;
}
