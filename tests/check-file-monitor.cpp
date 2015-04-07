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
#include <iostream>
#include "shmdata/unix-socket-server.hpp"
#include "shmdata/unix-socket-protocol.hpp"
#include "shmdata/file-monitor.hpp"

static const std::string socket_path("/tmp/check-file-monitor");
int main () {
  using namespace shmdata;

  // server protocol
  UnixSocketProtocol::onConnectDataMaker data(128,       // shm size
                                              8754,      // shm key to distribute
                                              "hello");  // user message
  UnixSocketProtocol::ServerSide sproto;
  sproto.get_connect_iov_ = [&data](){return data.get_connect_iov();};
  sproto.on_connect_cb_ = [](int) {};
  sproto.on_disconnect_cb_ = [](int) {};

  // testing
  {
    assert(!fileMonitor::is_unix_socket(socket_path));
    UnixSocketServer srv(socket_path, &sproto);
    assert(fileMonitor::is_unix_socket(socket_path));
    assert(srv);
  }
  assert(!fileMonitor::is_unix_socket(socket_path));
  return 0;
}

