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
#include "shmdata/unix-socket-protocol.hpp"

int main () {
  using namespace shmdata;

  // server protocol
  onConnectData data(128,       // shm size
                     8754,      // shm key to distribute
                     "hello");  // user message
  UnixSocketProtocol proto;
  proto.get_connect_iov_ = [&data](){return data.get_connect_iov();};
  proto.on_connect_cb_ = [](int d) { std::printf("(server) on_connect_cb\n");};
  proto.on_disconnect_cb_ = [](int d) { std::printf("(server) on_disconnect_cb\n");};

  // testing
  { std::printf("-- creation with not time to connect\n");
    UnixSocketServer srv("/tmp/check-unix-socket", &proto);
    UnixSocketClient cli("/tmp/check-unix-socket");
    assert(srv);
    assert(cli); }
  { std::printf("-- creation with not time to connect 2\n");
    UnixSocketClient cli("/tmp/check-unix-socket");
    UnixSocketServer srv("/tmp/check-unix-socket", &proto);
    assert(srv);
    assert(!cli); }
  { std::printf("-- client connects at creation\n");
    UnixSocketServer srv("/tmp/check-unix-socket", &proto);
    UnixSocketClient cli("/tmp/check-unix-socket");
    usleep(100000);
    assert(srv);
    assert(cli); }
  { std::printf("-- client can't connect at creation\n");
    UnixSocketClient cli("/tmp/check-unix-socket");
    UnixSocketServer srv("/tmp/check-unix-socket", &proto);
    usleep(100000);
    assert(srv);
    assert(!cli); }
  return 0;
}

