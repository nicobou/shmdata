/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

#undef NDEBUG

#include <sys/types.h>  // getuid & geteuid
#include <unistd.h>     // getuid & geteuid

#include <cassert>
#include <filesystem>
#include <thread>

#include "shmdata/console-logger.hpp"
#include "shmdata/follower.hpp"
#include "shmdata/writer.hpp"

namespace fs = std::filesystem;

int main()
{
  using namespace shmdata;
  ConsoleLogger logger;

  { // test writer initialisation fail with no right to write 
    Writer w("/tmp/check-unix-perms",
             1,
             "application/x-check-shmdata",
             &logger,
             nullptr,
             nullptr,
             0000);
    uid_t uid = getuid(), euid = geteuid();
    if (uid != 0 && uid == euid) {
      // current user do not have elevated privileges, writer must fail because of a permission
      // denied error
      assert(!w);
    }
  }
  
  { // test the rights are applied and a reader can read
    bool success = false;
    Writer w("/tmp/check-unix-perms",
             1,
             "application/x-check-shmdata",
             &logger,
             nullptr,
             nullptr,
             0666);
    assert(w);

    // test rights
    auto status = fs::status("/tmp/check-unix-perms");
    assert(fs::is_socket(status));
    auto perms = status.permissions();
    assert((perms & fs::perms::owner_read) != fs::perms::none);
    assert((perms & fs::perms::owner_write) != fs::perms::none);
    assert((perms & fs::perms::owner_exec) == fs::perms::none);
    assert((perms & fs::perms::group_read) != fs::perms::none);
    assert((perms & fs::perms::group_write) != fs::perms::none);
    assert((perms & fs::perms::group_exec) == fs::perms::none);
    assert((perms & fs::perms::others_read) != fs::perms::none);
    assert((perms & fs::perms::others_write) != fs::perms::none);
    assert((perms & fs::perms::others_exec) == fs::perms::none);

    // test reading
    Follower follower(
        "/tmp/check-unix-perms",
        [&](void*, size_t) { success = true; },
        nullptr,
        nullptr,
        &logger);

    auto i = 1;
    while (0 != i--) {
      assert(w.copy_to_shm(&i, sizeof(i)));
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    assert(success);

  } // end test rights are applied
  
  return 0;
}
