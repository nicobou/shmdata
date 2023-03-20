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
#include "switcher/utils/method-trait.hpp"
#include "switcher/utils/serialize-string.hpp"

int main() {
  {
    using namespace switcher;
    using namespace quiddity;

    Switcher::ptr manager = Switcher::make_switcher("test_manager");
    assert(quiddity::test::full(manager, "method-quid"));
    auto qrox = manager->quids<MPtr(&quiddity::Container::create)>("method-quid", "test", nullptr);
    auto mquid = qrox.get();
    assert(mquid);

    // testing "hello" method. Signature is string(string)
    using hello_meth_t = std::function<std::string(std::string)>;
    auto hello_id = mquid->meth<MPtr(&method::MBag::get_id)>("hello");
    assert(hello_id != 0);
    auto res = mquid->meth<MPtr(&method::MBag::invoke<hello_meth_t>)>(
        hello_id, std::make_tuple(std::string("Nicolas")));
    assert("hello Nicolas and count is 0" == res);

    // using "count" method. Signature is void()
    using count_meth_t = std::function<void()>;
    auto count_id = mquid->meth<MPtr(&method::MBag::get_id)>("count");
    assert(count_id != 0);
    mquid->meth<MPtr(&method::MBag::invoke<count_meth_t>)>(count_id, std::make_tuple());

    // testing count did its internal counting
    assert("hello Nicolas and count is 1" ==
           mquid->meth<MPtr(&method::MBag::invoke<hello_meth_t>)>(
               hello_id, std::make_tuple(std::string("Nicolas"))));

    // testing "many args" method: Signature bool(int, float, const std::string&, bool).
    // many return true only if arguments are <1,3.14,"is, but not ",false>
    using many_args_t = std::function<bool(int, float, const std::string&, bool)>;
    auto many_id = mquid->meth<MPtr(&method::MBag::get_id)>("many_args");
    assert(many_id != 0);
    assert(true == mquid->meth<MPtr(&method::MBag::invoke<many_args_t>)>(
                       many_id, std::make_tuple(1, 3.14f, std::string("is, but not "), false)));

    // testing "many args" invokation from string, using tuple deserialization.
    // Note that tuple are serialized
    auto tuple_from_str = deserialize::apply<method_trait<many_args_t>::args_t>(
        std::string("1,3.14,") + serialize::esc_for_tuple("is, but not ") + std::string(",false"));
    assert(tuple_from_str.first);  // first is a boolean indicating the success of deserialization
    assert(true ==
           mquid->meth<MPtr(&method::MBag::invoke<many_args_t>)>(many_id, tuple_from_str.second));

    assert(manager->quids<MPtr(&quiddity::Container::remove)>(qrox.get_id()));
  }  // end of scope is releasing the manager
  return 0;
}
