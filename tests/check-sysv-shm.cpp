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

#include "shmdata/unix-socket-server.hpp"
#include "shmdata/unix-socket-client.hpp"
#include "shmdata/unix-socket-protocol.hpp"
#include "shmdata/sysv-shm.hpp"


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
  UnixSocketServer srv("/tmp/check-unix-socket", &sproto);
  sysVShm shm("/tmp/check-unix-socket", 1, 1024, IPC_CREAT | 0666);
  if(!shm)
    return 1;
  return 0;
}

