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
#include "shmdata/writer.hpp"
#include "shmdata/reader.hpp"
#include "shmdata/console-logger.hpp"

/*
 this file contains several examples:
 - writer example that write to the shared memory by copying an existing buffer
   or writing directly into the memory
 - writer/reader with reader expecting to read buffer initial value
 */


// a struct with contiguous data storage 
using Frame = struct {
  size_t count{0};
  std::array<int, 3> data{{3, 1, 4}};
  // no vector ! vector.data is contiguous, but not vector
};

int main () {
  using namespace shmdata;
  ConsoleLogger logger;
  
  {  // writer example
    Writer w("/tmp/check-shmdata",
             sizeof(Frame),
             "application/x-check-shmdata",
             &logger);
    assert(w);

    {  // first method: copy the entire buffer 
      auto i = 300;
      Frame frame;
      while (0 != i--) {
        assert(w.copy_to_shm(&frame, sizeof(Frame)));
        frame.count++;
      }
    }  // end first method
   
    { // second method: access the memory and update only what need to be updated 
      size_t i = 300;
      Frame frame;
      // optional memory initialization with a copy
      assert(w.copy_to_shm(&frame, sizeof(Frame)));
      while (0 != --i) {
        //  the following is locking the shared memory for writing
        auto access = w.get_one_write_access();
        assert(access);
        auto frame = static_cast<Frame *>(access->get_mem());
        frame->count++;
        access->notify_clients(sizeof(Frame)); /* they will start reading
                                                  after w lock is release,
                                                  at destruction */
      }  // access is released, lock is freed
    }
  }// end writer example

  {  // copy writer with one reader
    Writer w("/tmp/check-shmdata",
             sizeof(Frame),
             "application/x-check-shmdata",
             &logger);
    assert(w);
    Reader r("/tmp/check-shmdata",
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
    assert(r);
    Frame frame;
    auto i = 300;
    while (0 != --i) {
      if (0 == i)
        std::cout << "i is 0" << std::endl;
      frame.count = i;
      assert(w.copy_to_shm(&frame, sizeof(Frame)));
    }
  }
  
    {  // direct access writer with one reader
    Writer w("/tmp/check-shmdata",
             sizeof(Frame),
             "application/x-check-shmdata",
             &logger);
    assert(w);
    // init
    {
      Frame frame;
      assert(w.copy_to_shm(&frame, sizeof(Frame)));
    }
    Reader r("/tmp/check-shmdata",
             [](void *data, size_t size){
               auto frame = static_cast<Frame *>(data);
               std::cout << "(direct access) new data for client "
                         << frame->count
                         << " (size " << size << ")"
                         << std::endl;
             },
             nullptr,
             nullptr,
             &logger);
    assert(r);
    auto i = 300;
    while (0 != --i) {
      //  the following is locking the shared memory for writing
      auto access = w.get_one_write_access();
      assert(access);
      auto frame = static_cast<Frame *>(access->get_mem());
      frame->count++;
      access->notify_clients(sizeof(Frame));
    }
  }

  return 0;
}

