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


/**
 * This test measures the latency between a write and a read from a Shmdata.
 * This is done as follows: writer measures curent time, and writes the value into the shmdata.
 * When called back, the reader mesures its curent time and compute the latency with the
 * time value read from the shmdata.
 *
 * The test succeed if the duration between a write and a read is less than a millisecond.
 * The actual duration, however, is more likely being around few microseconds.
 **/

#undef NDEBUG  // get assert in release mode

#include <cassert>
#include <array>
#include <iostream>
#include <thread>
#include <future>
#include "shmdata/writer.hpp"
#include "shmdata/follower.hpp"
#include "shmdata/console-logger.hpp"

using namespace shmdata;

int main () {
  using namespace shmdata;
  ConsoleLogger logger;

  {
    Writer w("/tmp/check-latency",
             1,
             "application/x-check-shmdata",
             &logger);
    assert(w);
 
    Follower follower("/tmp/check-latency",
                      [](void *data, size_t size){
                        // const auto reading_time = std::chrono::steady_clock::now().duration;
                        const auto now = std::chrono::steady_clock::now();
                        const auto reading_time =
                            std::chrono::duration_cast<std::chrono::microseconds, long int>(
                                now.time_since_epoch())
                                .count();
                        auto writing_time = static_cast<long int*>(data);
                        std::cout << "latency "
                                  << reading_time - *writing_time
                                  << "μs" 
                                  << std::endl;
                        // assert transmission is less than than a millisecond
                        assert(reading_time - *writing_time < 1000);
                      },
                      nullptr,
                      nullptr,
                      &logger);
    // testing 100 writes
    auto i = 100;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    while (0 != i--) {
      const auto start = std::chrono::steady_clock::now();
      const auto duration = std::chrono::duration_cast<std::chrono::microseconds, long int>(start.time_since_epoch()).count();
      assert(w.copy_to_shm(&duration, sizeof(long int)));
    }
  }
  return 0;
}
  
