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
static std::array<int, 64> data{{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
      12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
      28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43,
      44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
      60, 61, 62, 63}};

void writer(AbstractLogger *logger) {
    Writer w("/tmp/check-shm-resize",
             sizeof(int),
             "application/x-int-array",
             logger);
    assert(w);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto i = 1;
    while (i < 66) {
      assert(w.copy_to_shm(&data, i * sizeof(int)));
      i++;
    }
}

int main () {
  using namespace shmdata;
  ConsoleLogger logger;
  auto server_interactions = 0;
  {
    Follower follower("/tmp/check-shm-resize",
                      [](void *data, size_t size){
                        std::cout << "(copy) new data for client "
                                  << " (size " << size << ")"
                                  << std::endl;
                        auto frame = static_cast<int *>(data);
			for (size_t i = 0; i < size/sizeof(int); i++)
			  std::cout << frame[i] << ' ';
			std::cout << std::endl;
			    
                      },
                      [&](const std::string &){++server_interactions;},
                      [&](){++server_interactions;},
                      &logger);
    
    
    auto writer_handle = std::async(std::launch::async, writer, &logger);
    writer_handle.get();
  }
  
  return 0;
}

