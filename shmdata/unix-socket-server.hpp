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


#ifndef _SHMDATA_UNIX_SOCKET_SERVER_H_
#define _SHMDATA_UNIX_SOCKET_SERVER_H_

#include <future>
#include <vector>
#include <string>
#include <atomic>
#include "./safe-bool-idiom.hpp"
#include "./unix-socket.hpp"
#include "./unix-socket-protocol.hpp"

namespace shmdata{

class UnixSocketServer: public SafeBoolIdiom {
 public:
  UnixSocketServer(const std::string &path,
                   UnixSocketProtocol::ServerSide *proto,
                   int max_pending_cnx = 10);
  ~UnixSocketServer();
  UnixSocketServer() = delete;
  UnixSocketServer(const UnixSocketServer &) = delete;
  UnixSocketServer& operator=(const UnixSocketServer&) = delete;
  UnixSocketServer& operator=(UnixSocketServer&&) = default;

  // notify client and return the number of notification
  size_t notify_update();
  
 private:
  std::string path_;
  UnixSocket socket_{};
  int max_pending_cnx_;
  bool is_binded_{false};
  bool is_listening_{false};
  std::future<void> done_{};
  std::atomic_short quit_{0};
  std::vector<int> clients_{};
  UnixSocketProtocol::ServerSide *proto_;
  bool is_valid() const final;
  void client_interaction();
};

}  // namespace shmdata
#endif
