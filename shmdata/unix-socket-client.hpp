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


#ifndef _SHMDATA_UNIX_SOCKET_CLIENT_H_
#define _SHMDATA_UNIX_SOCKET_CLIENT_H_

#include <string>
#include <future>
#include <atomic>
#include "./safe-bool-idiom.hpp"
#include "./unix-socket.hpp"
#include "./unix-socket-protocol.hpp"

namespace shmdata{

class UnixSocketClient: public SafeBoolIdiom {
 public:
  UnixSocketClient(const std::string &path,
                   UnixSocketProtocol::ClientSide *proto);
  ~UnixSocketClient();
  UnixSocketClient() = delete;
  UnixSocketClient(const UnixSocketClient &) = delete;
  UnixSocketClient& operator=(const UnixSocketClient&) = delete;
  UnixSocketClient& operator=(UnixSocketClient&&) = default;
  
 private:
  std::string path_;
  UnixSocket socket_{};
  std::future<void> done_{};
  std::atomic_short quit_{0};
  bool connected_{false};
  bool is_valid_{false};
  UnixSocketProtocol::ClientSide *proto_;
  bool is_valid() const final;
  void server_interaction();
};

}  // namespace shmdata
#endif
