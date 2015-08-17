/*
 * Copyright (C) 2015 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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

#include <cassert>
#include <array>
#include <iostream>
#include <thread>
#include <future>
#include "shmdata/writer.hpp"
#include "shmdata/follower.hpp"
#include "shmdata/console-logger.hpp"

using namespace shmdata;

// a struct with contiguous data storage 
using Frame = struct {
  size_t count{0};
  std::array<int, 3> data{{3, 1, 4}};
  // no vector ! vector.data is contiguous, but not vector
};

int main () {
  using namespace shmdata;
  ConsoleLogger logger;

  {
    Writer w("/tmp/check-writer-follower",
             sizeof(Frame),
             "application/x-check-shmdata",
             &logger);
    assert(w);
    Frame frame;

    auto num_follow = 1;
    while(-1 != --num_follow){
      Follower follower("/tmp/check-writer-follower",
                        [](void *data, size_t size){
                          auto frame = static_cast<Frame *>(data);
                          std::cout << "(copy) new data for client "
                                    << frame->count
                                    << " (size " << size << ")"
                                    << std::endl;
                        },
                        nullptr,
                        nullptr,
                        &logger);
      auto i = 10;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      while (0 != i--) {
        assert(w.copy_to_shm(&frame, sizeof(Frame)));
        frame.count++;
      }
    }
    return 0;
  }
}
  
