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
#include "switcher/quiddity/quiddity-basic-test.hpp"
#include "switcher/switcher.hpp"

int main() {
  {
    using namespace switcher;

    Switcher::ptr manager = Switcher::make_switcher("test_manager");
    manager->factory<MPtr(&quid::Factory::scan_dir)>("./");

    // check can_sink_caps
    auto cqrox = manager->quids<MPtr(&quid::Container::create)>("cropper", "crop", nullptr);
    assert(cqrox);

    auto crop = cqrox.get();
    assert(crop);

    auto meth_id = crop->meth<MPtr(&MContainer::get_id)>("can-sink-caps");
    assert(0 != meth_id);

    // should reject connection if the caps is not "video/x-raw"
    bool can_connect = crop->meth<MPtr(&MContainer::invoke<std::function<bool(std::string)>>)>(
        meth_id, std::make_tuple("audio/x-raw"));

    assert(can_connect == false);

    // should accept connection if the caps are valid and of type "video/x-raw"
    can_connect = crop->meth<MPtr(&MContainer::invoke<std::function<bool(std::string)>>)>(
        meth_id,
        std::make_tuple("video/x-raw, format=I420, width=1920, height=1080, framerate=30/1"));

    assert(can_connect == true);

  }  // end of scope is releasing the manager
  return 0;
}
