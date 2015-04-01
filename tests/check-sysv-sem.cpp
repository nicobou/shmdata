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
#include <iostream>
#include "shmdata/sysv-sem.hpp"

using namespace shmdata;

static const std::array<int, 3> array{ {1,2,3} };

bool writer(sysVSem *sem, int *val){
  for (auto &it: array){
    writeLock wlock(sem);
    assert(wlock);
    *val = it;
    std::this_thread::sleep_for (std::chrono::milliseconds(10));
  }
  return true;
}

bool reader(sysVSem *sem, int *val){
  for (auto &it : array){
    readLock rlock(sem);
    assert(rlock);
    std::cout << *val << " " << it; 
    if (*val != it)
      std::cout << " error";
    std::cout << std::endl;
    assert (*val == it);
  }
  return true;
}


int main () {
  using namespace shmdata;
  {
    sysVSem sem(4312, IPC_CREAT | IPC_EXCL | 0666);
    assert(sem);
  }
  {
    sysVSem sem(4312, IPC_CREAT | IPC_EXCL | 0666);
    assert(sem);
    auto val = 0;
    auto writer_handle = std::async(std::launch::async,
                                    writer,
                                    &sem,
                                    &val);
    auto reader_handle1 = std::async(std::launch::async,
                                     reader,
                                     &sem,
                                     &val);
    auto reader_handle2 = std::async(std::launch::async,
                                     reader,
                                     &sem,
                                     &val);
    assert(writer_handle.get());
    assert(reader_handle1.get());
    assert(reader_handle2.get());
  }
  return 0;
}

