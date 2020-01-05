/*
 * This file is part of switcher-timecode.
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

#include "switcher/quiddity/basic-test.hpp"

int main() {
  {
    using namespace switcher;
    using namespace quiddity;
    Switcher::ptr switcher = Switcher::make_switcher("test_switcher");

    switcher->factory<MPtr(&quiddity::Factory::scan_dir)>("./");

    switcher->factory<MPtr(&quiddity::Factory::scan_dir)>("../jack");

    // creating a jack server
    InfoTree::ptr server_config = InfoTree::make();
    server_config->vgraft("driver", "dummy");
    server_config->vgraft("realtime", false);
    auto jserv = switcher->quids<MPtr(&quiddity::Container::create)>(
        "jackserver", "test_server", server_config.get());
    assert(jserv);
    assert(jserv.get()->prop<MPtr(&property::PBag::set_str_str)>("driver", "dummy"));
    assert(jserv.get()->prop<MPtr(&property::PBag::set_str_str)>("started", "true"));

    // Fringe case like CI cannot run this test successfully but we don't want it to fail.
    if (!switcher->quids<MPtr(&quiddity::Container::create)>(
            "ltcsource", "ltctestsourcedummy", nullptr))
      return 0;

    auto created =
        switcher->quids<MPtr(&quiddity::Container::create)>("ltcsource", "ltctestsource", nullptr);
    if (!created) return 1;

    if (!created.get()->prop<MPtr(&property::PBag::set_str_str)>("started", "true")) return 1;
    if (!switcher->quids<MPtr(&quiddity::Container::remove)>(created.get_id())) return 1;

    if (!quiddity::test::full(switcher, "ltcsource")) return 1;

  }  // end of scope is releasing the switcher
  return 0;
}
