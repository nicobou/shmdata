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
#include "shmdata/console-logger.hpp"

using namespace shmdata;

int main () {
  using namespace shmdata;
  ConsoleLogger log;
  {
    sysVSem sem(4312, &log, /* owner = */ true);
    assert(sem);
  }
  {
    sysVSem sem(4312, &log, /* owner = */ true);
    assert(sem);
    auto i = 65535;
    auto val = i;
    while (0 != i--){
      {
        // one writer, two readers
        {
          WriteLock wlock(&sem);
          // expecting two readers
          wlock.commit_readers(2);
          assert(wlock);
          val = i;
        }
        {  // first reader
          ReadLock rlock(&sem);
          assert(rlock);
          assert (val == i);
        }
        {  // second reader
          ReadLock rlock(&sem);
          assert(rlock);
          assert (val == i);
        }
      }
    }
  }
  return 0;
}

