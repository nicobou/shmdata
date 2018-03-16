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
#include "switcher/information-tree-json.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(MethodQuid,
                                     "method",
                                     "Example Method Plugin",
                                     "test",
                                     "",
                                     "Dummy plugin for testing/example purpose",
                                     "LGPL",
                                     "Nicolas Bouillot");

MethodQuid::MethodQuid(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      hello_id_(mmanage<MPtr(&MContainer::make_method<my_method_t>)>(
          "hello_",
          JSONSerializer::deserialize(
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
      count_id_(mmanage<MPtr(&MContainer::make_method<std::function<void()>>)>(
          "count_",
          JSONSerializer::deserialize(
              R"(
                  {
                   "name" : "Count Call",
                   "description" : "This Count Call method increments an internal counter by one when called",
                   "arguments" : null
                  }
              )"),
          [&]() { ++count_; })),
      many_args_id_(mmanage<MPtr(&MContainer::make_method<many_args_t>)>(
          "many_args_",
          JSONSerializer::deserialize(
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
                     },
                   ]
                  }
              )"),
          [&](int i, float f, const std::string& str, bool b) {
            debug("int %", std::to_string(i));
            debug("float %", std::to_string(f));
            debug("string %", str);
            debug("bool %", b ? std::string("true") : std::string("false"));
            return 1 == i && 3.14f == f && std::string("is, but not ") == str && b == false;
          })) {
  install_method("Hello World",                                  // long name
                 "hello-world",                                  // name
                 "say hello and repeat first argument",          // description
                 "the hello answer",                             // return description
                 Method::make_arg_description("Text To Repeat",  // first arg long name
                                              "text",            // fisrt arg name
                                              "string",          // first arg description
                                              nullptr),
                 (Method::method_ptr)&my_hello_world_method,
                 G_TYPE_STRING,
                 Method::make_arg_type_description(G_TYPE_STRING, nullptr),
                 this);

  debug("hello_id_ %", std::to_string(hello_id_));
}

gchar* MethodQuid::my_hello_world_method(gchar* first_arg, void* user_data) {
  MethodQuid* context = static_cast<MethodQuid*>(user_data);
  context->debug("hello world from myplugin");
  context->hello_ = std::string("hello ") + first_arg;
  // the g_free will be invoked by the method system:
  return g_strdup(context->hello_.c_str());
}

}  // namespace switcher
