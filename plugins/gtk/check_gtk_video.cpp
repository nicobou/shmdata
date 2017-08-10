/*
 * This file is part of switcher-gtk.
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

#include <unistd.h>  // sleep
#include "switcher/quiddity-basic-test.hpp"

#include <gdk/gdk.h>

int main() {
  {
    switcher::Switcher::ptr manager = switcher::Switcher::make_switcher("gtktest");

    manager->scan_directory_for_plugins("./");

    if (!gdk_display_get_default()) {
      // probably launched from ssh, could not find a display
      return 0;
    }

    // creating a "myplugin" quiddity
    if (manager->create("gtkwin", "win") != "win") {
      // cannot create gtk window, stopping the test
      return 1;
    }

    // creating a video source quiddity
    if (manager->create("videotestsrc", "vid").compare("vid") != 0) return 1;

    if (!manager->use_prop<MPtr(&switcher::PContainer::set_str_str)>("vid", "started", "true"))
      return 1;

    // usleep (1000000);

    // connecting
    if (!manager->invoke_va("win", "connect", nullptr, "/tmp/switcher_gtktest_vid_video", nullptr))
      return 1;

    usleep(500000);

    // removing quiddities
    if (!manager->remove("win")) return 1;

    if (!manager->remove("vid")) return 1;

    if (!switcher::QuiddityBasicTest::test_full(manager, "gtkvideosink")) return 1;
  }  // end of scope is releasing the manager

  return 0;  // success
}
