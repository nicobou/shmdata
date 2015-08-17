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


#ifndef _SHMDATA_UNIX_SOCKET_H_
#define _SHMDATA_UNIX_SOCKET_H_

#include <string>
#include "./safe-bool-idiom.hpp"
#include "./abstract-logger.hpp"

namespace shmdata{
class UnixSocketServer;
class UnixSocketClient;

class UnixSocket: public SafeBoolIdiom {
  friend UnixSocketServer;
  friend UnixSocketClient;
 public:
  UnixSocket(AbstractLogger *log);
  ~UnixSocket();
  UnixSocket(const UnixSocket &) = delete;
  UnixSocket& operator=(const UnixSocket&) = delete;
  UnixSocket& operator=(UnixSocket&&) = default;
  
 private:
  AbstractLogger *log_;
  int fd_{-1};
  bool is_valid() const final;
};

}  // namespace shmdata
#endif
