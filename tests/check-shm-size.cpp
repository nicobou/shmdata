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

#undef NDEBUG  // get assert in release mode

#include <cassert>
#include <array>
#include <iostream>
#include "shmdata/writer.hpp"
#include "shmdata/console-logger.hpp"
#include "shmdata/sysv-shm.hpp"

using namespace shmdata;


int main () {
  using namespace shmdata;
  ConsoleLogger logger;
  {
    unsigned long max_size = sysVShm::get_shmmax(&logger);
    // std::cout << max_size << '\n';
    // unsigned long mni = sysVShm::get_shmmni(&logger);
    // std::cout << mni << '\n';
    if (max_size > 268435456) //256MB
      max_size = 268435456;
    Writer w("/tmp/check-shm-size",
             max_size,
             "application/x-check-shmdata",
             &logger);
    assert(w);
  }
  return 0;
}
  
