/*
 * This file is part of switcher-plugin-example.
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

    manager->factory<MPtr(&quid::Factory::scan_dir)>("./");

    assert(test::full(manager, "custom-save"));
    manager->reset_state(true);

    InfoTree::ptr save;

    {  // check saving a cutom state
      auto test = manager->quids<MPtr(&quid::Container::create)>("custom-save", "test").get();
      assert(test);
      auto has_loaded_id =
          test->prop<MPtr(&PContainer::get_id)>(std::string("has_loaded_custom_state"));
      auto has_saved_id =
          test->prop<MPtr(&PContainer::get_id)>(std::string("has_saved_custom_state"));
      assert(!test->prop<MPtr(&PContainer::get<bool>)>(has_loaded_id));
      assert(!test->prop<MPtr(&PContainer::get<bool>)>(has_saved_id));

      // saving the custom state
      save = manager->get_state();
      assert(save);
      assert(!test->prop<MPtr(&PContainer::get<bool>)>(has_loaded_id));
      assert(test->prop<MPtr(&PContainer::get<bool>)>(has_saved_id));
    }

    // reset manager
    manager->reset_state(true);

    {  // check loading
      manager->load_state(save);
      auto loaded = manager->quids<MPtr(&quid::Container::get_qrox_from_name)>("test").get();
      assert(loaded);
      assert(loaded->prop<MPtr(&PContainer::get<bool>)>(
          loaded->prop<MPtr(&PContainer::get_id)>(std::string("has_loaded_custom_state"))));
      assert(!loaded->prop<MPtr(&PContainer::get<bool>)>(
          loaded->prop<MPtr(&PContainer::get_id)>(std::string("has_saved_custom_state"))));
    }

  }  // end of scope is releasing the manager
  return 0;
}
