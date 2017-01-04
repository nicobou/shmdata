/*
 * This file is part of switcher-myplugin.
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

#include "signal-quid.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(SignalQuid,
                                     "signal",
                                     "Signal Quiddity",
                                     "test",
                                     "",
                                     "Dummy quiddity for testing of signals",
                                     "LGPL",
                                     "Jérémie Soria");

SignalQuid::SignalQuid(const std::string&) {}

bool SignalQuid::init() {
  install_method("Emit Signal",                  // long name
                 "emit-signal",                  // name
                 "send \"test-signal\" signal",  // description
                 "No return",                    // return description
                 Method::make_arg_description("none", nullptr),
                 (Method::method_ptr)&my_signal_method,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_NONE, nullptr),
                 this);

  GType arg_type[] = {G_TYPE_STRING};
  install_signal_with_class_name("Quiddity",
                                 "On test signal",
                                 "test-signal",
                                 "A signal has been emitted",
                                 Signal::make_arg_description("Test signal quiddity",
                                                              "quiddity_name",
                                                              "the quiddity name",
                                                              "Method Name",
                                                              "method_name",
                                                              "the method name",
                                                              nullptr),
                                 1,
                                 arg_type);

  return true;
}

gboolean SignalQuid::my_signal_method(void*, void* user_data) {
  SignalQuid* context = static_cast<SignalQuid*>(user_data);
  context->signal_emit("test-signal");
  return true;
}
}