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
#include "switcher/infotree/information-tree-json.hpp"
#include "switcher/quiddity/quiddity-basic-test.hpp"

int main() {
  {
    using namespace switcher;
    auto test_name = std::string("check_jack");
    Switcher::ptr manager = Switcher::make_switcher(test_name);

    manager->factory<MPtr(&quid::Factory::scan_dir)>("./");

    // creating a jack server
    InfoTree::ptr server_config = InfoTree::make();
    server_config->vgraft("name", test_name);
    server_config->vgraft("driver", "dummy");
    server_config->vgraft("realtime", false);

    auto jserv = manager->quids<MPtr(&switcher::quid::Container::create)>(
        "jackserver", "test_server", server_config.get());
    assert(jserv);
    assert(jserv.get()->prop<MPtr(&PContainer::set_str_str)>("started", "true"));

    // run tests using this server
    InfoTree::ptr client_config = InfoTree::make();
    client_config->vgraft("server_name", test_name);
    assert(switcher::test::full(manager, "jacksink", client_config));
    assert(switcher::test::full(manager, "jacksrc", client_config));
    assert(switcher::test::full(manager, "jackserver", server_config));

  }  // end of scope is releasing the manager
  return 0;
}
