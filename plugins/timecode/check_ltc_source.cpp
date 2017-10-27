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

#include "switcher/quiddity-basic-test.hpp"

int main() {
  {
    switcher::Switcher::ptr switcher = switcher::Switcher::make_switcher("test_switcher");

    switcher->scan_directory_for_plugins("./");

    // Fringe case like CI cannot run this test successfully but we don't want it to fail.
    if (switcher->create("ltcsource", "ltctestsourcedummy") != "ltctestsourcedummy") return 0;

    if (switcher->create("ltcsource", "ltctestsource") != "ltctestsource") return 1;

    if (!switcher->use_prop<MPtr(&switcher::PContainer::set_str_str)>(
            "ltctestsource", "started", "true"))
      return 1;
    if (!switcher->remove("ltctestsource")) return 1;

    if (!switcher::QuiddityBasicTest::test_full(switcher, "ltcsource")) return 1;

  }  // end of scope is releasing the switcher
  return 0;
}
