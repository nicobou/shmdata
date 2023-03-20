/*
 * This file is part of switcher-plugin-example.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
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

#include "./method-quid.hpp"
#include "switcher/infotree/json-serializer.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(MethodQuid,
                                     "method-quid",
                                     "Example Quiddity with methods",
                                     "Dummy plugin for testing/example purpose",
                                     "LGPL",
                                     "Nicolas Bouillot");

MethodQuid::MethodQuid(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf)),
      hello_id_(mmanage<MPtr(&method::MBag::make_method<my_method_t>)>(
          "hello",
          infotree::json::deserialize(
              R"(
                  {
                   "name" : "Hello Method",
                   "description" : "This Hello method illustrates the use of quiddity methods",
                   "arguments" : [
                     {
                        "long name" : "Who are you ?",
                        "description" : "this will allow to call you by your name"
                     }
                   ]
                  }
              )"),
          [&](const std::string& str) {
            return std::string("hello ") + str + " and count is " + std::to_string(count_);
          })),
      count_id_(mmanage<MPtr(&method::MBag::make_method<std::function<void()>>)>(
          "count",
          infotree::json::deserialize(
              R"(
                  {
                   "name" : "Count Call",
                   "description" : "This Count Call method increments an internal counter by one when called",
                   "arguments" : null
                  }
              )"),
          [&]() { ++count_; })),
      many_args_id_(mmanage<MPtr(&method::MBag::make_method<many_args_t>)>(
          "many_args",
          infotree::json::deserialize(
              R"(
                  {
                   "name" : "Method With Many Arguments",
                   "description" : "This method illustrates the use of many arguments",
                   "arguments" : [
                     {
                        "long name" : "an int arg",
                        "description" : "first arg"
                     },
                     {
                        "long name" : "a float arg",
                        "description" : "second arg"
                     },
                     {
                        "long name" : "an string arg",
                        "description" : "third arg"
                     },
                     {
                        "long name" : "a bool arg",
                        "description" : "fourth arg"
                     }
                   ]
                  }
              )"),
          [&](int i, float f, const std::string& str, bool b) {
            sw_debug("int {:d}", i);
            sw_debug("float {:f}", f);
            sw_debug("string {}", str);
            sw_debug("bool {}", b);
            return 1 == i && 3.14f == f && std::string("is, but not ") == str && b == false;
          })) {
  sw_debug("hello_id_ {}", std::to_string(hello_id_));
}

}  // namespace quiddities
}  // namespace switcher
