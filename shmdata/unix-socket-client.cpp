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
#include <unistd.h>
#include <iostream>
#include "./unix-socket-client.hpp"

namespace shmdata{

UnixSocketClient::UnixSocketClient(const std::string &path,
                                   UnixSocketProtocol::ClientSide *proto) :
    path_(path),
    proto_(proto){
  if (!socket_)  // client not valid if socket is not valid
    return;
  if (nullptr == proto_)
    return;
  struct sockaddr_un sun;
  // fill socket address structure with server′s address
  memset(&sun, 0, sizeof(sun));
  sun.sun_family = AF_UNIX;
  strcpy(sun.sun_path, path.c_str());
  int len = offsetof(struct sockaddr_un, sun_path) + path_.size();
  if (0 != connect(socket_.fd_, (struct sockaddr *)&sun, len)) {
    perror("connect");
    return;
  }
  is_valid_ = true;
  done_ = std::async(std::launch::async,
                     [](UnixSocketClient *self){self->server_interaction();},
                     this);
}

UnixSocketClient::~UnixSocketClient() {
  if (done_.valid()) {
    quit_.store(1);
    done_.get();
  }
}

bool UnixSocketClient::is_valid() const {
  return is_valid_;
}


void UnixSocketClient::server_interaction() {
  fd_set allset;
  FD_ZERO(&allset);
  FD_SET(socket_.fd_, &allset);
  auto maxfd = socket_.fd_;
  struct timeval tv;  // select timeout
  size_t shm_size = 0;
  key_t shm_key = 0;
  char user_data[1000];
  struct iovec iov[3];
  iov[0].iov_base = &shm_size;
  iov[0].iov_len  = sizeof(size_t);
  iov[1].iov_base = &shm_key;
  iov[1].iov_len  = sizeof(key_t);
  iov[2].iov_base = user_data;
  iov[2].iov_len  = 1000;
  while (0 == quit_.load()) {
    // reset timeout since select may change values
    tv.tv_sec = 0;
    tv.tv_usec = 10000;  // 10 msec
    auto rset = allset;  /* rset gets modified each time around */
    if (select(maxfd + 1, &rset, NULL, NULL, &tv) < 0) {
      perror("select error");
      continue;
    }
    if (FD_ISSET(socket_.fd_, &rset)) {
      auto nread = readv(socket_.fd_, iov, 3);
      if (nread < 0) {
        perror("read");
      } else if (nread == 0) {
        std::printf("(client) server closed (?): fd %d\n", socket_.fd_);
        proto_->on_disconnect_cb_(socket_.fd_);
        FD_CLR(socket_.fd_, &allset);
      } else { /* process server′s message */
        std::cout << "(client) server message "
                  << shm_size
                  << "  "
                  << shm_key
                  << "  "
                  << std::string(user_data, 0, iov[2].iov_len)
                  << std::endl;
        // std::printf("server message: %s\n", buf);
        proto_->on_connect_cb_(socket_.fd_);
      }
    }
  }  // while (!quit_)
}


}  // namespace shmdata
