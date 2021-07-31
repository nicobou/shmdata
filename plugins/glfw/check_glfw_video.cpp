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
#include "switcher/quiddity/basic-test.hpp"
#include "switcher/utils/serialize-string.hpp"

int main() {
  {
    using namespace switcher;
    using namespace quiddity;
    using namespace claw;

    Switcher::ptr manager = Switcher::make_switcher("glfwtest");

    // // creating a video source quiddity
    // auto vqrox = manager->quids<MPtr(&quiddity::Container::create)>("videotestsrc", "vid",
    // nullptr); auto vid = vqrox.get(); assert(vid);

    // assert(vid->prop<MPtr(&property::PBag::set_str_str)>("started", "true"));

    // // creating a "glfwin" quiddity
    // auto wqrox = manager->quids<MPtr(&quiddity::Container::create)>("glfwin", "win", nullptr);
    // auto win = wqrox.get();
    // assert(win);

    // // connecting

    // auto sfid = win->claw<MPtr(&Claw::connect)>(win->claw<MPtr(&Claw::get_sfid)>("video"),
    //                                             vid->get_id(),
    //                                             vid->claw<MPtr(&Claw::get_swid)>("video"));
    // assert(Ids::kInvalid != sfid);
    // usleep(100000);
    // assert(win->claw<MPtr(&Claw::disconnect)>(sfid));

    // // We destroy it while connected to catch pipeline crashes.
    // sfid = win->claw<MPtr(&Claw::connect)>(win->claw<MPtr(&Claw::get_sfid)>("video"),
    //                                        vid->get_id(),
    //                                        vid->claw<MPtr(&Claw::get_swid)>("video"));
    // assert(Ids::kInvalid != sfid);

    // usleep(1000000);

    // assert(manager->quids<MPtr(&quiddity::Container::remove)>(wqrox.get_id()));
    // assert(manager->quids<MPtr(&quiddity::Container::remove)>(vqrox.get_id()));
    assert(quiddity::test::full(manager, "glfwin"));
  }  // end of scope is releasing the manager

  return 0;  // success
}
