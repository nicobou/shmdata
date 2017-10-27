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

SignalQuid::SignalQuid(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      signal_id_(smanage<MPtr(&SContainer::make)>("test-signal", "A test signal")) {
  install_method("Emit Signal",                  // long name
                 "emit-signal",                  // name
                 "send \"test-signal\" signal",  // description
                 "No return",                    // return description
                 Method::make_arg_description("none", nullptr),
                 (Method::method_ptr)&my_signal_method,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_NONE, nullptr),
                 this);
}

gboolean SignalQuid::my_signal_method(void*, void* user_data) {
  SignalQuid* context = static_cast<SignalQuid*>(user_data);
  auto tree = InfoTree::make();
  tree->graft(".zetremendouskey", InfoTree::make("zegreatvalue"));
  context->smanage<MPtr(&SContainer::notify)>(context->signal_id_, std::move(tree));
  // also grafting the tree
  context->graft_tree(".zetremendouskey", InfoTree::make("zegreatvalue"), true);
  context->prune_tree(".zetremendouskey", true);
  return true;
}
}
