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
#include <unordered_set>
#include <mutex>
#include <functional>
#include <future>
#include "./safe-bool-idiom.hpp"
#include "./unix-socket.hpp"
#include "./unix-socket-protocol.hpp"
#include "./abstract-logger.hpp"

namespace shmdata{

bool force_sockserv_cleaning(const std::string &path, AbstractLogger *log);

class UnixSocketServer: public SafeBoolIdiom {
 public:
  UnixSocketServer(const std::string &path,
                   UnixSocketProtocol::ServerSide *proto,
                   AbstractLogger *log,
                   std::function<void(int)> on_client_error = [](int){},
                   int max_pending_cnx = 10);
  ~UnixSocketServer();
  UnixSocketServer() = delete;
  UnixSocketServer(const UnixSocketServer &) = delete;
  UnixSocketServer& operator=(const UnixSocketServer&) = delete;
  UnixSocketServer& operator=(UnixSocketServer&&) = default;

  void start_serving();
  // return true if at least one notification has been sent
  short notify_update(size_t size = 0);
  
 private:
  AbstractLogger *log_;
  std::string path_;
  UnixSocket socket_;
  int max_pending_cnx_;
  bool is_binded_{false};
  bool is_listening_{false};
  std::future<void> done_{};
  std::atomic_short quit_{0};
  std::vector<int> clients_{};
  std::mutex clients_mutex_{};
  std::unordered_set<int> clients_notified_{};
  std::unordered_set<int> pending_clients_{};
  UnixSocketProtocol::ServerSide *proto_;
  std::function<void(int)> on_client_error_;
  bool is_valid() const final;
  void client_interaction();
};

}  // namespace shmdata
#endif
