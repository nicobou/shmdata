/*
 * This file is part of switcher-shmdelay.
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

#include <shmdata/console-logger.hpp>

#include "switcher/quiddity/basic-test.hpp"
#include "switcher/shmdata/follower.hpp"

bool success = false;
std::atomic<bool> do_continue{true};
std::condition_variable cond_var{};
std::mutex mut{};

using namespace switcher;
using namespace quiddity;
using namespace claw;

void wait_until_success() {
  // wait 3 seconds
  unsigned int count = 3;
  while (do_continue.load()) {
    std::unique_lock<std::mutex> lock(mut);
    if (count == 0) {
      do_continue.store(false);
    } else {
      --count;
      cond_var.wait_for(lock, std::chrono::seconds(1), []() { return !do_continue.load(); });
    }
  }
}

void notify_success() {
  std::unique_lock<std::mutex> lock(mut);
  success = true;
  do_continue.store(false);
  cond_var.notify_one();
}

int main() {
  {
    Switcher::ptr manager = Switcher::make_switcher("shmdelaytest");

    if (!quiddity::test::full(manager, "shmdelay")) return 1;

    auto shmdelaytest_qrox =
        manager->quids<MPtr(&quiddity::Container::create)>("shmdelay", "shmdelaytest", nullptr);
    if (!shmdelaytest_qrox) return 1;
    auto shmdelaytest = shmdelaytest_qrox.get();

    auto videotest_qrox =
        manager->quids<MPtr(&quiddity::Container::create)>("videotestsrc", "videotest", nullptr);
    if (!videotest_qrox) return 1;
    auto videotest = videotest_qrox.get();

    // We set 1 second of delay.
    if (!shmdelaytest->prop<MPtr(&quiddity::property::PBag::set_str_str)>("time_delay", "1000"))
      return 1;

    if (!videotest->prop<MPtr(&quiddity::property::PBag::set_str_str)>("started", "true")) return 1;

    if (!shmdelaytest->claw<MPtr(&quiddity::claw::Claw::connect)>(
            shmdelaytest->claw<MPtr(&quiddity::claw::Claw::get_sfid)>("shm"),
            videotest->get_id(),
            videotest->claw<MPtr(&quiddity::claw::Claw::get_swid)>("video")))
      return 1;

    auto start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();

    ::shmdata::ConsoleLogger logger;
    auto reader = std::make_unique<::shmdata::Follower>(
        shmdelaytest->claw<MPtr(&Claw::get_writer_shmpath)>(
            shmdelaytest->claw<MPtr(&Claw::get_swid)>("delayed-shm")),
        [&start_time](void*, size_t data_size) {
          if (!data_size) return;
          auto reception_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::system_clock::now().time_since_epoch())
                                    .count();
          // We are a bit more lenient with the 120ms margin because of the setup time of the test
          // and the CI possibly low performance.
          if (std::abs(reception_time - start_time - 1000) < 120) notify_success();
        },
        nullptr,
        nullptr,
        &logger);

    wait_until_success();

    if (!shmdelaytest->claw<MPtr(&quiddity::claw::Claw::disconnect)>(
            shmdelaytest->claw<MPtr(&quiddity::claw::Claw::get_sfid)>("shm")))
      return 1;

    videotest->prop<MPtr(&quiddity::property::PBag::set_str_str)>("started", "false");
    manager->quids<MPtr(&quiddity::Container::remove)>(videotest_qrox.get_id());
    manager->quids<MPtr(&quiddity::Container::remove)>(shmdelaytest_qrox.get_id());
  }  // end of scope is releasing the manager
  return success ? 0 : 1;
}
