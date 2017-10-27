/*
 * This file is part of switcher-portmidi.
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

#include <sys/stat.h>
#include <cstring>
#include <iostream>
#include "switcher/quiddity-basic-test.hpp"

int main() {
  bool success = true;
  {
    switcher::Switcher::ptr manager = switcher::Switcher::make_switcher("test_manager");

    manager->scan_directory_for_plugins("./");

    struct stat st;
    if (stat("/dev/snd", &st) == -1) {
      std::cerr << "Could not open /dev/snd in MIDI test: " << strerror(errno) << '\n';
      return 0;
    }

    if (manager->create("midisrc", "src").compare("src") == 0)
      manager->remove("src");
    else
      success = false;

    if (manager->create("midisrc", "sink").compare("sink") == 0)
      manager->remove("sink");
    else
      success = false;

    if (!switcher::QuiddityBasicTest::test_full(manager, "midisrc")) success = false;

    if (!switcher::QuiddityBasicTest::test_full(manager, "midisink")) success = false;
  }  // end of scope is releasing the manager

  if (success)
    return 0;
  else
    return 1;
}
