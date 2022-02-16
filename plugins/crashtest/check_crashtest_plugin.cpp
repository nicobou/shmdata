/*
 * This file is part of switcher-crashtest.
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

#include <signal.h>

#include <cassert>
#include <vector>

#include "switcher/quiddity/basic-test.hpp"

static void on_segfault(int, siginfo_t*, void*) { exit(0); }

int main() {
  {
    using namespace switcher;
    using namespace quiddity;

    Switcher::ptr manager = Switcher::make_switcher("crashtest");
    assert(quiddity::test::full(manager, "crashtest"));
    auto qrox = manager->quids<MPtr(&quiddity::Container::create)>("crashtest", "test", nullptr);
    auto quid = qrox.get();
    assert(quid);

    // handle SEGFAULT in order to provide success
    struct sigaction sa;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_NODEFER;
    sa.sa_sigaction = on_segfault;
    sigaction(SIGSEGV, &sa, NULL); /* ignore whether it works or not */

    // generate the SEGFAULT
    quid->prop<MPtr(&property::PBag::set_str_str)>("crash", "true");

  }          // end of scope is releasing the manager
  return 1;  // error, SEGFAULT should have been handled
}
