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
#include <vector>
#include "switcher/property-container.hpp"
#include "switcher/quiddity-basic-test.hpp"

int main() {
  {
    using namespace switcher;
    Switcher::ptr manager = Switcher::make_switcher("test_manager");

    manager->scan_directory_for_plugins("./");

    assert(QuiddityBasicTest::test_full(manager, "dummy"));

    // creating a "myplugin" quiddity
    assert(manager->create("dummy", "test") == "test");

    assert(manager->use_prop<MPtr(&PContainer::set_str_str)>("test", "color_", "0A93D8FF"));
    assert(!manager->use_prop<MPtr(&PContainer::set_str_str)>("test", "color_", "0A93D8"));
    assert(!manager->use_prop<MPtr(&PContainer::set_str_str)>("test", "color_", "GGGGGGGGGG"));
    assert(!manager->use_prop<MPtr(&PContainer::set_str_str)>("test", "color_", "0000000T"));

    // testing hello-world method
    std::string* res = nullptr;
    assert(manager->invoke_va("test", "hello-world", &res, "Nico", nullptr));
    assert(*res == "hello Nico");
    delete res;

    // removing the quiddity
    assert(manager->remove("test"));
  }  // end of scope is releasing the manager
  return 0;
}
