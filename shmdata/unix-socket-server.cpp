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
#include <stddef.h>
#include <algorithm>
#include "./unix-socket-server.hpp"

namespace shmdata{

UnixSocketServer::UnixSocketServer(const std::string &path, int max_pending_cnx) :
    path_(path),
    max_pending_cnx_(max_pending_cnx) {
  if (!socket_)  // server not valid if socket is not valid
    return;
  struct sockaddr_un sock_un;
  if (path_.size() >= sizeof(sock_un.sun_path)) {
    errno = ENAMETOOLONG;
    perror("name");
    return;
  }
  unlink (path_.c_str());
  memset(&sock_un, 0, sizeof(sock_un));
  sock_un.sun_family = AF_UNIX;
  strcpy(sock_un.sun_path, path_.c_str());
  printf("%d, %s\n", socket_.fd_, path_.c_str());
  if (bind(socket_.fd_, (struct sockaddr *) &sock_un, sizeof(struct sockaddr_un)) < 0) {
    perror("bind");
    return;
  } else {
    is_binded_ = true;
  }
  if (listen (socket_.fd_, max_pending_cnx_) < 0) {
    perror("listen");
    return;
  } else {
    is_listening_ = true;
  }
  FD_ZERO(&allset_);
  FD_SET(socket_.fd_, &allset_);
  done_ = std::async(std::launch::async,
                     [](UnixSocketServer *self){self->io_multiplex();},
                     this);
}

UnixSocketServer::~UnixSocketServer() {
  if (done_.valid()) {
    quit_.store(1);
    // // fake connect in order to restart select
    // struct sockaddr_un sun;
    // memset(&sun, 0, sizeof(sun));
    // sun.sun_family = AF_UNIX;
    // strcpy(sun.sun_path, path_.c_str());
    // int len = offsetof(struct sockaddr_un, sun_path) + path_.size();
    // connect(socket_.fd_, (struct sockaddr *)&sun, len);
    done_.get();
    unlink (path_.c_str());
  }
}

bool UnixSocketServer::is_valid() const {
  return is_binded_ && is_listening_;
}

void UnixSocketServer::io_multiplex() {
  auto maxfd = socket_.fd_;
  int clifd = -1;
  char	buf[1000];  // MAXLINE
  while (0 == quit_.load()) {
    auto rset = allset_;  /* rset gets modified each time around */
    if (select(maxfd + 1, &rset, NULL, NULL, NULL) < 0) {
      perror("select error");
      continue;
    }
    if (FD_ISSET(socket_.fd_, &rset)) {
      // accept new client request
      auto clifd = accept(socket_.fd_, NULL, NULL);
      if (clifd < 0)
        perror("accept");
      FD_SET(clifd, &allset_);
      if (clifd > maxfd)
        maxfd = clifd;  // max fd for select()
      clients_[clifd] = 0;
      std::printf("new connection: fd %d\n", clifd);
      continue;
    }
    for (auto &it : clients_) {
      if (FD_ISSET(it.first, &rset)) {
        auto nread = read(it.first, buf, 1000);  // MAXLINE
        if (nread < 0) {
          perror("read");
        } else if (nread == 0) {
          std::printf("closed: fd %d\n", it.first);
          clients_.erase(it.first);
          FD_CLR(it.first, &allset_);
          close(it.first);
        } else { /* process client′s request */
          std::printf("client request\n");
        }
      }
    }
  }  // while (!quit_)
}

}  // namespace shmdata
