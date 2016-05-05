/*
 * This file is part of libswitcher.
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

#include <gst/gst.h>
#include <cassert>
#include <iostream>
#include <string>
#include <vector>
#include "switcher/quiddity-manager.hpp"

static int signal_counter = 0;

void quiddity_created_removed_cb(const std::string& /*subscriber_name */,
                                 const std::string& /*quiddity_name */,
                                 const std::string& signal_name,
                                 const std::vector<std::string>& params,
                                 void* /*user_data */) {
  std::printf("%s: %s", signal_name.c_str(), params[0].c_str());
  signal_counter++;
}

int main() {
  {
    switcher::QuiddityManager::ptr manager =
        switcher::QuiddityManager::make_manager("testing_signals");

    // make on-quiddity-created and on-quiddity-removed signals
    assert("create_remove_spy" ==
           manager->create("create_remove_spy", "create_remove_spy"));
    assert(manager->make_signal_subscriber(
        "signal_subscriber", quiddity_created_removed_cb, manager.get()));
    assert(manager->subscribe_signal(
        "signal_subscriber", "create_remove_spy", "on-quiddity-created"));
    assert(manager->subscribe_signal(
        "signal_subscriber", "create_remove_spy", "on-quiddity-removed"));

    assert("vid1" == manager->create("videotestsrc", "vid1"));
    assert("fake1" == manager->create("fakesink", "fake1"));
    assert("vid2" == manager->create("videotestsrc", "vid2"));
    assert("fake2" == manager->create("fakesink", "fake2"));

    auto subscribers = manager->list_signal_subscribers();
    assert(subscribers.size() == 1);
    assert(subscribers[0] == "signal_subscriber");

    std::vector<std::pair<std::string, std::string>> signals =
        manager->list_subscribed_signals("signal_subscriber");
    assert(signals.size() == 2);
    assert(signals.at(0).first == "create_remove_spy");
    assert(signals.at(0).second == "on-quiddity-created");
    assert(signals.at(1).first == "create_remove_spy");
    assert(signals.at(1).second == "on-quiddity-removed");

    assert(manager->remove("create_remove_spy"));
    assert(manager->list_subscribed_signals("signal_subscriber").size() == 0);
    assert(manager->remove_signal_subscriber("signal_subscriber"));
  }

  gst_deinit();
  if (signal_counter == 4)  // 4 creations has been asked
    return 0;
  return 1;
}
