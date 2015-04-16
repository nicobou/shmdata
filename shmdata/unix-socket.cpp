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

UnixSocket::UnixSocket() :
    fd_(socket(AF_UNIX, SOCK_STREAM, 0)){
  if (-1 == fd_)
    perror("socket");
  int flags = fcntl(fd_, F_GETFL, 0);
  if (flags < 0)
    perror("fcntl(F_GETFL)");
  if (fcntl(fd_, F_SETFL, flags | O_NONBLOCK | FD_CLOEXEC) < 0)
     perror("fcntl(F_SETFL)");
#ifdef SO_NOSIGPIPE
  int set = 1;
  setsockopt(fd_, SOL_SOCKET, SO_NOSIGPIPE, (void *)&set, sizeof(int));
#endif
}

UnixSocket::~UnixSocket() {
  if(is_valid()) {
    close(fd_);
  }
}
bool UnixSocket::is_valid() const {
  return -1 != fd_;
}

}  // namespace shmdata
