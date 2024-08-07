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

#undef NDEBUG  // get assert in release mode

#include <cassert>
#include <cstring>  // memcpy
#include <array>
#include <iostream>
#include <thread>
#include <future>
#include "shmdata/writer.hpp"
#include "shmdata/follower.hpp"
#include "shmdata/console-logger.hpp"

using namespace shmdata;

// a struct with contiguous data storage 
static std::array<int, 64> data{{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
        12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
        28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43,
        44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
        60, 61, 62, 63, 64}};

void writer(AbstractLogger *logger) {
    Writer w("/tmp/check-shm-resize",
             1,
             "application/x-int-array",
             logger);
    assert(w);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::cout << "********* copy_to_shm" << std::endl;
    auto i = 1;
    while (i < 30) {
      auto newsize = i * sizeof(int);
      assert(w.copy_to_shm(&data, newsize));
      assert(w.alloc_size() == newsize);
      i++;
    }
    std::cout << "********* get_one_write_access_ptr_resize" << std::endl;
    while (i < 40) {
      auto newsize = i * sizeof(int);
      OneWriteAccess *access = w.get_one_write_access_ptr_resize(newsize);
      assert(access);
      assert(w.alloc_size() == newsize);
      std::memcpy(access->get_mem(), &data, newsize);
      access->notify_clients(newsize);
      delete(access);
      i++;
    }
    std::cout << "********* get_one_write_access_resize" << std::endl;
    while (i < 60) { 
      auto newsize = i * sizeof(int);
      auto access = w.get_one_write_access_resize(newsize);
      assert(access);
      assert(w.alloc_size() == newsize);
      std::memcpy(access->get_mem(), &data, newsize);
      access->notify_clients(newsize);
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
			for (size_t i = 0; i < size/sizeof(int); i++) {
			  std::cout << frame[i] << ' ';
                          assert(static_cast<int>(i + 1) == frame[i]);
                        }
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

