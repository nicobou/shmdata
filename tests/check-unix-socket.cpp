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
#include "shmdata/console-logger.hpp"

int main () {

  using namespace shmdata;
  ConsoleLogger logger;
  // server protocol

  UnixSocketProtocol::ServerSide sproto(
      [](int id) {std::printf("(server) on_connect_cb, id %d\n", id);},
      [](int id) {std::printf("(server) on_disconnect_cb, id %d\n", id);},
      [](){return
            UnixSocketProtocol::onConnectData (128,          // shm size
                                               "hello");});  // user message
            

  // client protocol
  UnixSocketProtocol::ClientSide cproto(
      [&cproto](){
        std::cout << "(client) on_connect_cb "
                  << " shm_size " << cproto.data_.shm_size_ 
                  << " user_data " << cproto.data_.user_data_.data()
                  << std::endl;
      },
      [](){ std::printf("(client) on_disconnect_cb\n"); },
      [](size_t){ std::printf("(client) on_update_cb\n"); });

  // testing
  { std::printf("-- creation with not time to connect\n");
    UnixSocketServer srv("/tmp/check-unix-socket", &sproto, &logger);
    srv.start_serving();
    UnixSocketClient cli("/tmp/check-unix-socket", &logger);
    assert(srv);
    assert(cli);
    cli.start(&cproto);
  }
  { std::printf("-- creation with not time to connect 2\n");
    UnixSocketClient cli("/tmp/check-unix-socket", &logger);
    UnixSocketServer srv("/tmp/check-unix-socket", &sproto, &logger);
    srv.start_serving();
    assert(srv);
    assert(!cli);
  }
  { std::printf("-- clients connects at creation\n");
    UnixSocketServer srv("/tmp/check-unix-socket", &sproto, &logger);
    srv.start_serving();
    UnixSocketClient cli1("/tmp/check-unix-socket", &logger);
    UnixSocketClient cli2("/tmp/check-unix-socket", &logger);
    UnixSocketClient cli3("/tmp/check-unix-socket", &logger);
    assert(srv);
    assert(cli1);
    cli1.start(&cproto);
    assert(cli2);
    cli2.start(&cproto);
    assert(cli3);
    cli3.start(&cproto);
    usleep(100000);
    auto i = 5;
    while (0 != i--) {
      srv.notify_update();
      usleep(10000);
    }
  }
  { std::printf("-- client can't connect at creation\n");
    UnixSocketClient cli("/tmp/check-unix-socket", &logger);
    UnixSocketServer srv("/tmp/check-unix-socket", &sproto, &logger);
    srv.start_serving();
    usleep(100000);
    assert(srv);
    assert(!cli); }
  return 0;
}

