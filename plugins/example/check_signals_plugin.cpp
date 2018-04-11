/*
 * This file is part of switcher-plugin-example.
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

#include <gst/gst.h>
#include <cassert>
#include <vector>
#include "switcher/switcher.hpp"

unsigned int signal_counter = 0;

int main() {
  using namespace switcher;
  Switcher::ptr manager = Switcher::make_switcher("testing_signals");
  manager->factory<MPtr(&quid::Factory::scan_dir)>("./");

  // Test create/remove notification system
  unsigned int create_remove_counter = 0;
  manager->quids<MPtr(&quid::Container::register_creation_cb)>(
      [&](auto) { ++create_remove_counter; });
  manager->quids<MPtr(&quid::Container::register_removal_cb)>(
      [&](auto) { ++create_remove_counter; });
  auto qrox = manager->quids<MPtr(&quid::Container::create)>("signal", "signal-quiddity");
  auto qsig = qrox.get();
  assert(qsig);
  
  auto registration_id = qsig->sig<MPtr(&SContainer::subscribe_by_name)>(
      "test-signal",
      [&](const InfoTree::ptr&) { ++signal_counter; });

  assert(0 != registration_id);

  auto emit_signal_id = qsig->meth<MPtr(&MContainer::get_id)>("emit-signal");
  qsig->meth<MPtr(&MContainer::invoke_str)>(emit_signal_id, std::string());
  qsig->meth<MPtr(&MContainer::invoke_str)>(emit_signal_id, std::string());
  qsig->meth<MPtr(&MContainer::invoke_str)>(emit_signal_id, std::string());

  assert(qsig->sig<MPtr(&SContainer::unsubscribe_by_name)>("test-signal", registration_id));
  // the following should not imply incrementation of signal_counter
  qsig->meth<MPtr(&MContainer::invoke_str)>(emit_signal_id, std::string());

  assert(manager->quids<MPtr(&quid::Container::remove)>(qrox.get_id()));

  gst_deinit();
  if (create_remove_counter == 2 && signal_counter == 3) return 0;
  return 1;
}
