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

  {  // writer example
    // Writer w("/tmp/check-shmdata",
    //          sizeof(Frame),
    //          "application/x-check-shmdata");
    // assert(w);

    // {  // first method: copy the entire buffer 
    //   auto i = 300;
    //   Frame frame;
    //   while (0 != i--) {
    //     assert(w.copy_to_shm(&frame, sizeof(Frame)));
    //     frame.count++;
    //   }
    // }  // end first method
    // std::cout << __FUNCTION__ << " " << __LINE__ << std::endl;

   
    // FIXME 
    // { // second method: access the memory and update only what need to be updated 
    //   size_t i = 300;
    //   Frame frame;
    //   // optional memory initialization with a copy
    //   assert(w.copy_to_shm(&frame, sizeof(Frame)));
    //   while (0 != --i) {
    //     //  the following is locking the shared memory for writing
    //     auto access = w.get_one_write_access();
    //     assert(access);
    //     auto frame = static_cast<Frame *>(access->get_mem());
    //     frame->count++;
    //   }  // access is released, lock is freed
    // }
  }// end writer example

  {  // copy writer with one reader
    std::cout << __FUNCTION__ << " " << __LINE__ << std::endl;
    Writer w("/tmp/check-shmdata",
             sizeof(Frame),
             "application/x-check-shmdata");
    std::cout << __FUNCTION__ << " " << __LINE__ << std::endl;
    assert(w);
    std::cout << __FUNCTION__ << " " << __LINE__ << std::endl;
    Reader r("/tmp/check-shmdata",
             [](void *data){
               auto frame = static_cast<Frame *>(data);
               std::cout << "(copy) new data for client "
                         << frame->count
                         << std::endl;
             });
    std::cout << __FUNCTION__ << " " << __LINE__ << std::endl;
    assert(r);
    std::cout << __FUNCTION__ << " " << __LINE__ << std::endl;
    Frame frame;
    auto i = 300;
    while (0 != --i) {
      if (0 == i)
        std::cout << "i is 0" << std::endl;
      frame.count = i;
      assert(w.copy_to_shm(&frame, sizeof(Frame)));
    }
  }
  return 0;
  
    {  // direct access writer with one reader
    Writer w("/tmp/check-shmdata",
             sizeof(Frame),
             "application/x-check-shmdata");
    assert(w);
    // init
    {
      Frame frame;
      assert(w.copy_to_shm(&frame, sizeof(Frame)));
    }
    Reader r("/tmp/check-shmdata",
             [](void *data){
               auto frame = static_cast<Frame *>(data);
               std::cout << "(direct access) new data for client "
                         << frame->count
                         << std::endl;
             });
    assert(r);
    auto i = 300;
    while (0 != --i) {
      //  the following is locking the shared memory for writing
      auto access = w.get_one_write_access();
      assert(access);
      auto frame = static_cast<Frame *>(access->get_mem());
      frame->count++;
    }
  }

  return 0;
}

