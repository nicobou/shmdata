/*
 * This file is part of switcher-gsoap.
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

#include <unistd.h>
#include "switcher/infotree/json-serializer.hpp"
#include "switcher/quiddity/basic-test.hpp"

int main() {
  bool success = false;
  {
    switcher::Switcher::ptr manager = switcher::Switcher::make_switcher("test_manager");

    manager->factory<MPtr(&switcher::quiddity::Factory::scan_dir)>("./");

    if (switcher::quiddity::test::full(manager, "SOAPcontrolServer")) success = true;

  }  // end of scope is releasing the manager

  if (success)
    return 0;
  else
    return 1;
}
