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

#include <unistd.h>  // usleep
#include <cassert>
#include <iostream>
#include "shmdata/unix-socket-server.hpp"
#include "shmdata/unix-socket-client.hpp"

int main () {
  { std::printf("-- creation with not time to connect\n");
    shmdata::UnixSocketServer srv("/tmp/check-unix-socket");
    shmdata::UnixSocketClient cli("/tmp/check-unix-socket");
    assert(srv);
    assert(cli); }
  { std::printf("-- creation with not time to connect 2\n");
    shmdata::UnixSocketClient cli("/tmp/check-unix-socket");
    shmdata::UnixSocketServer srv("/tmp/check-unix-socket");
    assert(srv);
    assert(!cli); }
  { std::printf("-- client connects at creation\n");
    shmdata::UnixSocketServer srv("/tmp/check-unix-socket");
    shmdata::UnixSocketClient cli("/tmp/check-unix-socket");
    usleep(100000);
    assert(srv);
    assert(cli); }
  { std::printf("-- client can't connect at creation\n");
    shmdata::UnixSocketClient cli("/tmp/check-unix-socket");
    shmdata::UnixSocketServer srv("/tmp/check-unix-socket");
    usleep(100000);
    assert(srv);
    assert(!cli); }
  return 0;
}

