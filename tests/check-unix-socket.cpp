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
  UnixSocketProtocol::onConnectDataMaker data(128,       // shm size
                                              8754,      // shm key to distribute
                                              "hello");  // user message
  UnixSocketProtocol::ServerSide sproto;
  sproto.get_connect_iov_ = [&data](){return data.get_connect_iov();};
  sproto.on_connect_cb_ = [](int id) {std::printf("(server) on_connect_cb, id %d\n", id);};
  sproto.on_disconnect_cb_ = [](int id) {std::printf("(server) on_disconnect_cb, id %d\n", id);};

  // client protocol
  UnixSocketProtocol::ClientSide cproto;
  cproto.on_connect_cb_ = [&cproto](int id){
    std::cout << "(client) on_connect_cb, id "
    << id
    << " shm_size " << cproto.data_.shm_size_ 
    << " shm_key " << cproto.data_.shm_key_
    << " user_data " << cproto.data_.get_user_data().c_str()
    << std::endl;
  };
  cproto.on_disconnect_cb_ = [](int d) {std::printf("(client) on_disconnect_cb, id %d\n", d);};

  // testing
  { std::printf("-- creation with not time to connect\n");
    UnixSocketServer srv("/tmp/check-unix-socket", &sproto);
    UnixSocketClient cli("/tmp/check-unix-socket", &cproto);
    assert(srv);
    assert(cli); }
  { std::printf("-- creation with not time to connect 2\n");
    UnixSocketClient cli("/tmp/check-unix-socket", &cproto);
    UnixSocketServer srv("/tmp/check-unix-socket", &sproto);
    assert(srv);
    assert(!cli); }
  { std::printf("-- clients connects at creation\n");
    UnixSocketServer srv("/tmp/check-unix-socket", &sproto);
    UnixSocketClient cli1("/tmp/check-unix-socket", &cproto);
    UnixSocketClient cli2("/tmp/check-unix-socket", &cproto);
    UnixSocketClient cli3("/tmp/check-unix-socket", &cproto);
    assert(srv);
    assert(cli1);
    assert(cli2);
    assert(cli3);
    usleep(100000); }
  { std::printf("-- client can't connect at creation\n");
    UnixSocketClient cli("/tmp/check-unix-socket", &cproto);
    UnixSocketServer srv("/tmp/check-unix-socket", &sproto);
    usleep(100000);
    assert(srv);
    assert(!cli); }
  return 0;
}

