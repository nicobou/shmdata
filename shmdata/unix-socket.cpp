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

#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>  // perror
#include <unistd.h>
#include <fcntl.h>
#include <sys/un.h>
#include <string>
#include "./unix-socket.hpp"

namespace shmdata{

UnixSocket::UnixSocket(const std::string &path) :
    path_(path),
    socket_(socket(AF_UNIX, SOCK_STREAM, 0)){
  if (-1 == socket_)
    perror("socket");
  int flags = fcntl(socket_, F_GETFL, 0);
  if (flags < 0)
    perror("fcntl(F_GETFL)");
  if (fcntl(socket_, F_SETFL, flags | O_NONBLOCK | FD_CLOEXEC) < 0)
     perror("fcntl(F_SETFL)");
   struct sockaddr_un sock_un;
   sock_un.sun_family = AF_UNIX;
   strncpy(sock_un.sun_path, path_.c_str(), sizeof(sock_un.sun_path) - 1);
   if (bind(socket_, (struct sockaddr *) &sock_un, sizeof(struct sockaddr_un)) < 0)
     perror("bind");
   if (listen (socket_, 10) < 0)  // max 10 pending connections 
     perror("listen");
}

UnixSocket::~UnixSocket() {
  if(is_valid()) {
    close(socket_);
    unlink (path_.c_str());
  }
}

bool UnixSocket::is_valid() const {
  return -1 != socket_;  // FIXME should be also binded 
}

}  // namespace shmdata
