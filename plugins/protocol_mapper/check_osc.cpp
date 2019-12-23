/*
 * This file is part of switcher-curl.
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
#include "switcher/quiddity/quiddity-basic-test.hpp"

int main() {
  {
    using namespace switcher;

    Switcher::ptr manager = Switcher::make_switcher("test_manager");
    manager->factory<MPtr(&quid::Factory::scan_dir)>("./");
    assert(switcher::test::full(manager, "protocol-mapper"));
    auto created =
        manager->quids<MPtr(&quid::Container::create)>("protocol-mapper", std::string(), nullptr);
    assert(created);
    auto quid = created.get();
    assert(quid->prop<MPtr(&PContainer::set_str_str)>("config_file", "protocol-osc.json"));
    assert(quid->prop<MPtr(&PContainer::set_str_str)>("int32", "true"));
    assert(quid->prop<MPtr(&PContainer::set_str_str)>("wrongtype", "true"));
    assert(quid->prop<MPtr(&PContainer::set_str_str)>("notype", "true"));
    assert(quid->prop<MPtr(&PContainer::set_str_str)>("ardour_goto_start", "true"));
  }
  return 0;
}
