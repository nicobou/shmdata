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
#include <iostream>
#include <vector>
#include "switcher/property-container.hpp"
#include "switcher/quiddity-basic-test.hpp"

int main() {
  {
    using namespace switcher;

    Switcher::ptr manager = Switcher::make_switcher("test_manager");

    manager->scan_directory_for_plugins("./");

    assert(QuiddityBasicTest::test_full(manager, "custom-save"));
    manager->reset_state(true);

    // creating a "myplugin" quiddity
    assert(manager->create("custom-save", "test") == "test");
    assert(manager->create("dummy", "dummy") == "dummy");  // dummy does not use custom state save
    auto has_loaded_id = manager->use_prop<MPtr(&PContainer::get_id)>(
        "test", std::string("has_loaded_custom_state"));
    auto has_saved_id =
        manager->use_prop<MPtr(&PContainer::get_id)>("test", std::string("has_saved_custom_state"));
    assert(!manager->use_prop<MPtr(&PContainer::get<bool>)>("test", has_loaded_id));
    assert(!manager->use_prop<MPtr(&PContainer::get<bool>)>("test", has_saved_id));

    // saving the custom state
    auto save = manager->get_state();
    assert(save);
    assert(!manager->use_prop<MPtr(&PContainer::get<bool>)>("test", has_loaded_id));
    assert(manager->use_prop<MPtr(&PContainer::get<bool>)>("test", has_saved_id));

    // reset manager
    manager->reset_state(true);

    // load the saved file
    manager->load_state(save);
    assert(manager->use_prop<MPtr(&PContainer::get<bool>)>("test", has_loaded_id));
    assert(!manager->use_prop<MPtr(&PContainer::get<bool>)>("test", has_saved_id));

  }  // end of scope is releasing the manager
  return 0;
}
