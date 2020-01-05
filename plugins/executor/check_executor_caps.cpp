/*
 * This file is part of switcher-executor.
 *
 * switcher-executor is free software; you can redistribute it and/or
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
#include "switcher/quiddity/basic-test.hpp"
#include "switcher/switcher.hpp"

int main() {
  {
    using namespace switcher;
    using namespace quiddity;

    Switcher::ptr manager = Switcher::make_switcher("test_manager");
    manager->factory<MPtr(&quiddity::Factory::scan_dir)>("./");

    // check can_sink_caps
    auto eqrox = manager->quids<MPtr(&quiddity::Container::create)>("executor", "exec", nullptr);
    assert(eqrox);

    auto exec = eqrox.get();
    assert(exec);

    auto meth_id = exec->meth<MPtr(&method::MBag::get_id)>("can-sink-caps");
    assert(0 != meth_id);

    // should accept connection if empty
    assert("" == exec->prop<MPtr(&property::PBag::get_str_str)>("whitelist_caps"));

    bool can_connect = exec->meth<MPtr(&method::MBag::invoke<std::function<bool(std::string)>>)>(
        meth_id, std::make_tuple("test"));

    assert(can_connect == true);

    // should reject connection if the caps doesn't match
    assert(exec->prop<MPtr(&property::PBag::set_str_str)>("whitelist_caps", "audio/x-raw"));
    assert("audio/x-raw" == exec->prop<MPtr(&property::PBag::get_str_str)>("whitelist_caps"));

    can_connect = exec->meth<MPtr(&method::MBag::invoke<std::function<bool(std::string)>>)>(
        meth_id, std::make_tuple("test"));

    assert(can_connect == false);

    // should accept connection if the caps match
    can_connect = exec->meth<MPtr(&method::MBag::invoke<std::function<bool(std::string)>>)>(
        meth_id, std::make_tuple("audio/x-raw"));

    assert(can_connect == true);

    // should accept connection if the caps match with many whitelisted caps
    assert(exec->prop<MPtr(&property::PBag::set_str_str)>("whitelist_caps",
                                                          "audio/x-raw, video/x-raw"));

    can_connect = exec->meth<MPtr(&method::MBag::invoke<std::function<bool(std::string)>>)>(
        meth_id, std::make_tuple("video/x-raw"));

    assert(can_connect == true);

  }  // end of scope is releasing the manager
  return 0;
}
