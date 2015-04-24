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

void writer(AbstractLogger *logger) {
    Writer w("/tmp/check-follower",
             sizeof(Frame),
             "application/x-check-shmdata",
             logger);
    assert(w);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto i = 10;
    Frame frame;
    while (0 != i--) {
      assert(w.copy_to_shm(&frame, sizeof(Frame)));
      frame.count++;
    }
}

int main () {
  using namespace shmdata;
  ConsoleLogger logger;
  auto server_interactions = 0;
  auto num_successive_write = 5;
  {
    Follower follower("/tmp/check-follower",
                      [](void *data, size_t size){
                        auto frame = static_cast<Frame *>(data);
                        std::cout << "(copy) new data for client "
                                  << frame->count
                                  << " (size " << size << ")"
                                  << std::endl;
                      },
                      [&](const std::string &){++server_interactions;},
                      [&](){++server_interactions;},
                      &logger);
    
   
    auto i = num_successive_write;
    while (-1 != --i) {
      auto writer_handle = std::async(std::launch::async, writer, &logger);
      writer_handle.get();
      //std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  if (server_interactions == 2 * num_successive_write)
    return 0;
  return 1;  // fail
}

