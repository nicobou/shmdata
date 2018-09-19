/*
 * This file is part of switcher-glfwin.
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

#include <GLFW/glfw3.h>
#include <unistd.h>  // sleep
#include <cassert>
#include <iostream>
#include "switcher/quiddity-basic-test.hpp"
#include "switcher/serialize-string.hpp"

int main() {
  {
    using namespace switcher;
    Switcher::ptr manager = Switcher::make_switcher("glfwtest");

    manager->factory<MPtr(&quid::Factory::scan_dir)>("./");

    if (!glfwInit()) {
      // probably launched from ssh, could not find a display
      return 0;
    }

    // creating a video source quiddity
    auto vqrox = manager->quids<MPtr(&quid::Container::create)>("videotestsrc", "vid");
    auto vid = vqrox.get();
    assert(vid);

    assert(vid->prop<MPtr(&PContainer::set_str_str)>("started", "true"));

    // creating a "glfwin" quiddity
    auto wqrox = manager->quids<MPtr(&quid::Container::create)>("glfwin", "win");
    auto win = wqrox.get();
    assert(win);

    // connecting
    auto connect_id = win->meth<MPtr(&MContainer::get_id)>("connect");
    assert(0 != connect_id);
    auto disconnect_id = win->meth<MPtr(&MContainer::get_id)>("disconnect");
    assert(0 != disconnect_id);
    assert(win->meth<MPtr(&MContainer::invoke_str)>(
        connect_id, serialize::esc_for_tuple(vid->make_shmpath("video"))));
    usleep(100000);
    assert(win->meth<MPtr(&MContainer::invoke_str)>(
        disconnect_id, serialize::esc_for_tuple(vid->make_shmpath("video"))));

    // We destroy it while connected to catch pipeline crashes.
    assert(win->meth<MPtr(&MContainer::invoke_str)>(
        connect_id, serialize::esc_for_tuple(vid->make_shmpath("video"))));

    usleep(1000000);

    assert(manager->quids<MPtr(&quid::Container::remove)>(wqrox.get_id()));
    assert(manager->quids<MPtr(&quid::Container::remove)>(vqrox.get_id()));
    assert(test::full(manager, "glfwin"));
  }  // end of scope is releasing the manager

  return 0;  // success
}
