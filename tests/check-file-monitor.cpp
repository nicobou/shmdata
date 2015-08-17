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
#include "shmdata/console-logger.hpp"

static const std::string socket_path("/tmp/check-file-monitor");
int main () {
  using namespace shmdata;
  ConsoleLogger logger;
  UnixSocketProtocol::ServerSide sproto(
      [](int id) {std::printf("(server) on_connect_cb, id %d\n", id);},
      [](int id) {std::printf("(server) on_disconnect_cb, id %d\n", id);},
      [](){return UnixSocketProtocol::onConnectData (128,         // shm size
                                                     "hello");}); // user message
  
  // testing
  {
    assert(!fileMonitor::is_unix_socket(socket_path, &logger));
    UnixSocketServer srv(socket_path, &sproto, &logger);
    srv.start_serving();
    assert(fileMonitor::is_unix_socket(socket_path, &logger));
    assert(srv);
  }
  assert(!fileMonitor::is_unix_socket(socket_path, &logger));
  return 0;
}

