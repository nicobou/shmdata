/*
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#undef NDEBUG  // get assert in release mode

#include <gst/gst.h>

#include <atomic>
#include <cassert>
#include <condition_variable>

#include "switcher/infotree/information-tree.hpp"  // remove me
#include "switcher/quiddity/basic-test.hpp"
#include "switcher/switcher.hpp"

using namespace switcher;

static bool success = false;
static std::atomic<bool> do_continue{true};
static std::condition_variable cond_var{};
static std::mutex mut{};

void wait_until_success() {
  // wait 3 seconds
  unsigned int count = 30;
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
    // making the switcher
    Switcher::ptr manager = Switcher::make_switcher("bundle");

    // loading configuration with bundle
    assert(manager->conf<MPtr(&Configuration::from_file)>("./check_bundle.config"));

    // creating and removing some complex bundles
    auto bundles = {"source-bundle", "sink-bundle", "filter-bundle", "whitespaces-bundle"};
    for (const auto& bundle : bundles) {
      assert(quiddity::test::create(manager, bundle));
    }

    // testing shmdata communication from a bundle to an other
    auto srcqrox =
        manager->quids<MPtr(&quiddity::Container::create)>("vid-source-bundle", "src", nullptr);
    assert(srcqrox);
    auto dummyqrox =
        manager->quids<MPtr(&quiddity::Container::create)>("dummy-sink-bundle", "dummy", nullptr);
    assert(dummyqrox);
    auto dummy = dummyqrox.get();

    assert(srcqrox.get()->prop<MPtr(&quiddity::property::PBag::set_str_str)>("started", "true"));
    assert(0 !=
           dummy->prop<MPtr(&quiddity::property::PBag::subscribe)>(
               dummy->prop<MPtr(&quiddity::property::PBag::get_id)>("dummy/frame-received"), [&]() {
                 if (dummy->prop<MPtr(&quiddity::property::PBag::get<bool>)>(
                         dummy->prop<MPtr(&quiddity::property::PBag::get_id)>(
                             "dummy/frame-received"))) {
                   notify_success();
                 }
               }));

    std::cout << srcqrox.get()->conspec<MPtr(&InfoTree::get_copy)>()->serialize_json() << '\n';
    std::cout << dummy->conspec<MPtr(&InfoTree::get_copy)>()->serialize_json() << '\n';
    assert(dummy->claw<MPtr(&quiddity::claw::Claw::connect)>(
        dummy->claw<MPtr(&quiddity::claw::Claw::get_sfid)>("dummy/default"),
        srcqrox.get_id(),
        srcqrox.get()->claw<MPtr(&quiddity::claw::Claw::get_swid)>("encoder/video-encoded")));
    wait_until_success();

    assert(srcqrox.get()->prop<MPtr(&quiddity::property::PBag::set_str_str)>("started", "false"));
    if (!success) {
      std::cout << "No data received." << std::endl;
      return 1;
    }
  }
  gst_deinit();
  return 0;
}
