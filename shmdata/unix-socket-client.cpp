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

#include <sys/un.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stddef.h>
#include "./unix-socket-client.hpp"

namespace shmdata{

UnixSocketClient::UnixSocketClient(const std::string &path) :
    path_(path) {
  if (!socket_)  // client not valid if socket is not valid
    return;
  struct sockaddr_un sun;
  // fill socket address structure with server′s address
  memset(&sun, 0, sizeof(sun));
  sun.sun_family = AF_UNIX;
  strcpy(sun.sun_path, path.c_str());
  int len = offsetof(struct sockaddr_un, sun_path) + path_.size();
  if (0 != connect(socket_.fd_, (struct sockaddr *)&sun, len) < 0) {
    perror("connect");
    return;
  }
  is_valid_ = true;
  // done_ = std::async(std::launch::async,
  //                    [](UnixSocketClient *self){self->io_multiplex();},
  //                    this);
}

UnixSocketClient::~UnixSocketClient() {
  // if (done_.valid())
  //   done_.get();
}

bool UnixSocketClient::is_valid() const {
  return is_valid_;
}

}  // namespace shmdata
