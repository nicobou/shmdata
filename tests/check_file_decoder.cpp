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

#include "switcher/quiddity-basic-test.hpp"
#include "switcher/shmdata-follower.hpp"

bool success = false;
std::atomic<bool> do_continue{true};
std::condition_variable cond_var{};
std::mutex mut{};

using namespace switcher;

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
    switcher::Switcher::ptr manager = switcher::Switcher::make_switcher("filedecoder");

    if (!switcher::test::full(manager, "filesrc")) return 1;

    auto filesrc =
        manager->quids<MPtr(&switcher::quid::Container::create)>("filesrc", "src", nullptr).get();

    ::shmdata::ConsoleLogger logger;
    auto reader = std::make_unique<shmdata::Follower>(filesrc->make_shmpath("audio"),
                                                      [](void*, size_t data_size) {
                                                        if (!data_size) return;
                                                        notify_success();
                                                      },
                                                      nullptr,
                                                      nullptr,
                                                      &logger);

    filesrc->prop<MPtr(&PContainer::set_str_str)>("loop", "true");
    filesrc->prop<MPtr(&PContainer::set_str_str)>("play", "true");
    filesrc->prop<MPtr(&PContainer::set_str_str)>("location", "./oie.mp3");

    wait_until_success();

  }  // end of scope is releasing the manager
  return success ? 0 : 1;
}
