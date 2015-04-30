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

using namespace shmdata;

static std::atomic_int done(0);
static ConsoleLogger logger;

// a struct with contiguous data storage 
using Frame = struct {
  size_t count{0};
  std::array<int, 3> data{{3, 1, 4}};
  // no vector ! vector.data is contiguous, but not vector
};

bool reader(){
  { // creating one reader
    std::this_thread::sleep_for (std::chrono::milliseconds(10));
    Reader r("/tmp/check-stress",
             [](void *data, size_t size){
               // auto frame = static_cast<Frame *>(data);
               // std::cout << "(one reader) new data for client "
               //           << frame->count
               //           << " (size " << size << ")"
               //           << std::endl;
             },
             nullptr,
             nullptr,
             &logger);
    assert(r);
    std::cout << "one more reader" << std::endl;
    std::this_thread::sleep_for (std::chrono::milliseconds(1000));
  }
  std::cout << "one less reader" << std::endl;
  { // creating five readers
    std::this_thread::sleep_for (std::chrono::milliseconds(10));
    Reader r1("/tmp/check-stress",
              [](void *data, size_t size){
                // auto frame = static_cast<Frame *>(data);
                // std::cout << "(1) new data for client "
                //           << frame->count
                //           << " (size " << size << ")"
                //           << std::endl;
              },
             nullptr,
             nullptr,
              &logger);
    assert(r1);
    Reader r2("/tmp/check-stress",
              [](void *data, size_t size){
                // auto frame = static_cast<Frame *>(data);
                // std::cout << "(2) new data for client "
                //           << frame->count
                //           << " (size " << size << ")"
                //           << std::endl;
              },
             nullptr,
             nullptr,
             &logger);
    assert(r2);
    Reader r3("/tmp/check-stress",
              [](void *data, size_t size){
                // auto frame = static_cast<Frame *>(data);
                // std::cout << "(3) new data for client "
                //           << frame->count
                //           << " (size " << size << ")"
                //           << std::endl;
              },
             nullptr,
             nullptr,
             &logger);
    assert(r3);
    Reader r4("/tmp/check-stress",
              [](void *data, size_t size){
                // auto frame = static_cast<Frame *>(data);
                // std::cout << "(4) new data for client "
                //           << frame->count
                //           << " (size " << size << ")"
                //           << std::endl;
              },
              nullptr,
              nullptr,
              &logger);
    assert(r4);
    Reader r5("/tmp/check-stress",
              [](void *data, size_t size){
                // auto frame = static_cast<Frame *>(data);
                // std::cout << "(5) new data for client "
                //           << frame->count
                //          << " (size " << size << ")"
                //           << std::endl;
              },
              nullptr,
              nullptr,
              &logger);
    assert(r5);
    std::cout << "five more reader" << std::endl;
    std::this_thread::sleep_for (std::chrono::milliseconds(1000));
  }
  std::this_thread::sleep_for (std::chrono::milliseconds(100));
  done.store(1);
  std::cout << "done set to 1" << std::endl;
  return true;
}

int main () {
  using namespace shmdata;

  {
    // direct access writer with one reader
    Writer w("/tmp/check-stress",
             sizeof(Frame),
             "application/x-check-shmdata",
             &logger);
    assert(w);
    // init
    {
      Frame frame;
      assert(w.copy_to_shm(&frame, sizeof(Frame)));
    }
    Reader r("/tmp/check-stress",
             [](void *data, size_t size){
               // auto frame = static_cast<Frame *>(data);
               // std::cout << "(0) new data for client "
               //           << frame->count
               //           << " (size " << size << ")"
               //           << std::endl;
             },
             nullptr,
             nullptr,
             &logger);
    std::cout << "one reader" << std::endl;
    assert(r);
    auto reader_handle = std::async(std::launch::async, reader);
    while (1 != done.load()) {
      //  the following is locking the shared memory for writing
      auto access = w.get_one_write_access();
      assert(access);
      access->notify_clients(sizeof(Frame));
      auto frame = static_cast<Frame *>(access->get_mem());
      frame->count++;
    }
    assert(reader_handle.get());
  }
  std::this_thread::sleep_for (std::chrono::milliseconds(1000));
  std::cout << "fin" << std::endl;
  return 0;
}

