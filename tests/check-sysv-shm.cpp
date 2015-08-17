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
#include "shmdata/sysv-shm.hpp"
#include "shmdata/console-logger.hpp"

int main () {
  using namespace shmdata;
  ConsoleLogger logger;
  {
    sysVShm shm(1234, 1024, &logger, /*owner = */ true);
    assert(shm);
  }
  {
    sysVShm shm(1234, 1024, &logger, /*owner = */ true);
    assert(shm);
  }
  { //write read
    sysVShm shm_writer(1234, sizeof(int), &logger, /*owner = */ true);
    assert(shm_writer);
    sysVShm shm_reader(1234, sizeof(int), &logger, /*owner = */ false);
    assert(shm_reader);
    *(int *)shm_writer.get_mem() = 7;
    assert(7 == *(int *)shm_reader.get_mem());
  }
  
  return 0;
}

