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

#include <cassert>
#include <vector>
#include "switcher/quiddity/basic-test.hpp"
#include "switcher/quiddity/property/pbag.hpp"
#include "switcher/utils/serialize-string.hpp"

int main() {
  {
    using namespace switcher;
    using namespace quiddity;

    Switcher::ptr manager = Switcher::make_switcher("test_manager");


    assert(quiddity::test::full(manager, "property-quid"));

    // creating a "myplugin" quiddity
    auto qrox = manager->quids<MPtr(&quiddity::Container::create)>("property-quid", "test", nullptr);
    auto pquid = qrox.get();
    assert(pquid);

    assert(pquid->prop<MPtr(&property::PBag::set_str_str)>("color_", "0A93D8FF"));
    assert(!pquid->prop<MPtr(&property::PBag::set_str_str)>("color_", "0A93D8"));
    assert(!pquid->prop<MPtr(&property::PBag::set_str_str)>("color_", "GGGGGGGGGG"));
    assert(!pquid->prop<MPtr(&property::PBag::set_str_str)>("color_", "0000000T"));

    // tuple testing (get)
    using MyTuple = std::tuple<long long, float, std::string>;
    auto tid = pquid->prop<MPtr(&property::PBag::get_id)>("tuple_");
    std::cout << pquid->prop<MPtr(&property::PBag::get_str)>(tid) << '\n';

    MyTuple my_tuple = pquid->prop<MPtr(&property::PBag::get<MyTuple>)>(tid);
    std::cout << "get is working !!!"
              << " " << std::get<0>(my_tuple) << " "  // 2
              << std::get<1>(my_tuple) << " "         // 2.2
              << std::get<2>(my_tuple) << "\n";       // a22

    // tuple testing (set)
    pquid->prop<MPtr(&property::PBag::set_str)>(
        tid, std::string("4,4.4,") + serialize::esc_for_tuple("b,44"));

    std::cout << pquid->prop<MPtr(&property::PBag::get_str)>(tid) << '\n';

    my_tuple = pquid->prop<MPtr(&property::PBag::get<MyTuple>)>(tid);
    std::cout << "get after set"
              << " " << std::get<0>(my_tuple) << " "  // 4
              << std::get<1>(my_tuple) << " "         // 4.4
              << std::get<2>(my_tuple) << "\n";       // b,44

    // removing the quiddity
    assert(manager->quids<MPtr(&quiddity::Container::remove)>(qrox.get_id()));
  }  // end of scope is releasing the manager
  return 0;
}
