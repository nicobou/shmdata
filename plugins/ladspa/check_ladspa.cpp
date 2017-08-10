/*
 * This file is part of switcher-ladspa.
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

#include <assert.h>
#include <atomic>
#include <switcher/information-tree-json.hpp>
#include "switcher/quiddity-basic-test.hpp"

using namespace switcher;

static bool success = false;
static std::atomic<bool> do_continue{true};
static std::condition_variable cond_var{};
static std::mutex mut{};

void wait_until_success() {
  // wait 3 seconds
  uint count = 3;
  while (do_continue) {
    std::unique_lock<std::mutex> lock(mut);
    if (count == 0) {
      do_continue = false;
    } else {
      --count;
      cond_var.wait_for(lock, std::chrono::seconds(1), []() { return !do_continue.load(); });
    }
  }
}

void notify_success() {
  std::unique_lock<std::mutex> lock(mut);
  success = true;
  do_continue = false;
  cond_var.notify_one();
}

int main() {
  {
    Switcher::ptr manager = Switcher::make_switcher("ladspatest");

    manager->scan_directory_for_plugins("./");

    manager->load_configuration_file("./check_ladspa.json");

    // creating a ladspa audiotest bundle
    auto bundle = manager->create("audiotestladspa");
    assert(!bundle.empty());

    if (!manager->use_prop<MPtr(&PContainer::set_str_str)>(bundle.c_str(), "started", "true"))
      return 1;

    manager->use_prop<MPtr(&PContainer::subscribe)>(
        bundle,
        manager->use_prop<MPtr(&PContainer::get_id)>(bundle, "dummy/frame-received"),
        [&]() {
          if (manager->use_prop<MPtr(&PContainer::get<bool>)>(
                  bundle,
                  manager->use_prop<MPtr(&PContainer::get_id)>(bundle, "dummy/frame-received"))) {
            notify_success();
          }
        });

    wait_until_success();

    if (!success) {
      std::cout << "No data received." << std::endl;
      return 1;
    }

    if (!manager->remove(bundle)) return 1;

    if (!QuiddityBasicTest::test_full(manager, "ladspa")) return 1;
  }  // end of scope is releasing the manager

  return 0;  // success
}
