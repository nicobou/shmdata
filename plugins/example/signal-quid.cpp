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

#include "signal-quid.hpp"
#include "switcher/infotree/information-tree-json.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(SignalQuid,
                                     "signal",
                                     "Signal Quiddity",
                                     "test",
                                     "",
                                     "Dummy quiddity for testing of signals",
                                     "LGPL",
                                     "Jérémie Soria");

SignalQuid::SignalQuid(quid::Config&& conf)
    : Quiddity(std::forward<quid::Config>(conf)),
      signal_id_(smanage<MPtr(&SContainer::make)>("test-signal", "A test signal")) {
  mmanage<MPtr(&MContainer::make_method<std::function<bool()>>)>(
      "emit-signal",
      infotree::json::deserialize(
          R"(
                  {
                   "name" : "Emit Signal",
                   "description" : "send \"test-signal\" signal",
                   "arguments" : []
                  }
              )"),
      [&]() {
        auto tree = InfoTree::make();
        tree->graft(".zetremendouskey", InfoTree::make("zegreatvalue"));
        smanage<MPtr(&SContainer::notify)>(signal_id_, std::move(tree));
        // also grafting the tree
        graft_tree(".zetremendouskey", InfoTree::make("zegreatvalue"), true);
        prune_tree(".zetremendouskey", true);
        return true;
      });
}

}  // namespace quiddities
}  // namespace switcher
