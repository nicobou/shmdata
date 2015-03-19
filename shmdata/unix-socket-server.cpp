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
#include "./unix-socket-server.hpp"

namespace shmdata{

UnixSocketServer::UnixSocketServer(const std::string &path, int max_pending_cnx) :
    path_(path),
    max_pending_cnx_(max_pending_cnx) {
  if (!socket_)  // server not valid if socket is not valid
    return;
  struct sockaddr_un sock_un;
  sock_un.sun_family = AF_UNIX;
  strncpy(sock_un.sun_path, path_.c_str(), sizeof(sock_un.sun_path) - 1);
  if (bind(socket_.fd_, (struct sockaddr *) &sock_un, sizeof(struct sockaddr_un)) < 0)
    perror("bind");
  else
    is_binded_ = true;
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
  if (done_.valid())
    done_.get();
  if(is_valid()) {
    unlink (path_.c_str());
    
  }
}

bool UnixSocketServer::is_valid() const {
  return is_binded_ && is_listening_;
}

void UnixSocketServer::io_multiplex() {
  int maxfd = socket_.fd_;
  while (!quit_) {
    fd_set rset = allset_;  /* rset gets modified each time around */
    if (select(maxfd + 1, &rset, NULL, NULL, NULL) < 0)
      perror("select error");
    if (FD_ISSET(socket_.fd_, &rset)) {
      // accept new client request
      int clifd = accept(socket_.fd_, NULL, NULL);
      if (clifd < 0)
        perror("accept");
      FD_SET(clifd, &allset_);
      if (clifd > maxfd)
        maxfd = clifd;  // max fd for select()
      std::printf("new connection: fd %d", clifd);
      continue;
    }
    
    // for (i = 0; i <= maxi; i++) {   /* go through client[] array */
    //   if ((clifd = client[i].fd) < 0)
    //     continue;
    //   if (FD_ISSET(clifd, &rset)) {
    //     /* read argument buffer from client */
    //     if ((nread = read(clifd, buf, MAXLINE)) < 0) {
    //       log_sys("read error on fd %d", clifd);
    //     } else if (nread == 0) {
    //       log_msg("closed: uid %d, fd %d",
    //               client[i].uid, clifd);
    //       client_del(clifd);  /* client has closed cxn */
    //       FD_CLR(clifd, &allset);
    //       close(clifd);
    //     } else {    /* process client′s request */
    //       handle_request(buf, nread, clifd, client[i].uid);
    //     }
    //   }
    // }

  }
}

}  // namespace shmdata
