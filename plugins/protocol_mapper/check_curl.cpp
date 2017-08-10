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
#include "switcher/quiddity-basic-test.hpp"

int main() {
  {
    using namespace switcher;

    Switcher::ptr manager = Switcher::make_switcher("test_manager");
    manager->scan_directory_for_plugins("./");

    assert(switcher::QuiddityBasicTest::test_full(manager, "protocol-mapper"));
    auto quid = manager->create("protocol-mapper");
    assert(manager->use_prop<MPtr(&PContainer::set_str_str)>(
        quid, "config_file", "protocol-curl.json"));
    assert(manager->use_prop<MPtr(&PContainer::set_str_str)>(quid, "continuous_message", "true"));
    assert(manager->use_prop<MPtr(&PContainer::set_str_str)>(quid, "bang", "true"));
    assert(manager->use_prop<MPtr(&PContainer::set_str_str)>(quid, "wrong_url", "true"));
    assert(manager->use_prop<MPtr(&PContainer::set_str_str)>(quid, "test_timeout", "true"));
  }
  return 0;
}
