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
#include "shmdata/writer.hpp"

// a struct with contiguous data storage 
using Frame = struct {
  size_t count{0};
  std::array<int, 3> data{{3, 1, 4}};
};

int main () {
  using namespace shmdata;

  writer w("/tmp/check-shmdata",
           sizeof(Frame),
           "check data format");
  assert(w);
  auto i = 300;
  Frame frame;
  while (0 != i--) {
    assert(w.copy_to_shm(&frame, sizeof(Frame)));
    frame.count++;
  }
  return 0;
}

