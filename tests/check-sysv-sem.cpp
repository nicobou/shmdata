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
#include <future>
#include <atomic>
#include <iostream>
#include "shmdata/sysv-sem.hpp"

using namespace shmdata;

static std::atomic_int done(0);

static const int init_val = 65535;

bool writer(sysVSem *sem, int *val){
  auto i = 65535;
  while (0 != i--){
    {
      WriteLock wlock(sem);
      assert(wlock);
      *val = i;
    }
  }
  return true;
}

bool reader(sysVSem *sem, int *val){
  {
    auto previous = init_val;
    while (0 == done.load()){
      ReadLock rlock(sem);
      assert(rlock);
      
      assert (*val <= previous);
      previous = *val;
    }
  }
  return true;
}


int main () {
  using namespace shmdata;
  {
    sysVSem sem(4312, /* owner = */ true);
    assert(sem);
  }
  {
    sysVSem sem(4312, /* owner = */ true);
    assert(sem);
    auto val = init_val;
    // wait readers lunched in order
    auto reader_handle = std::async(std::launch::async,
                                    reader,
                                    &sem,
                                    &val);
    auto reader_handle2 = std::async(std::launch::async,
                                     reader,
                                     &sem,
                                     &val);
    auto writer_handle = std::async(std::launch::async,
                                    writer,
                                    &sem,
                                    &val);
    assert(writer_handle.get());
    done.store(1);  // kills readers
    assert(reader_handle.get());
    assert(reader_handle2.get());
  }
  return 0;
}

