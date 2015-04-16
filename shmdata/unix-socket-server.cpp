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
#include <sys/uio.h>
#include <algorithm>
#include <vector>
#include <iostream> // debug
#include "./unix-socket-server.hpp"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL SO_NOSIGPIPE
#endif

namespace shmdata{

UnixSocketServer::UnixSocketServer(const std::string &path,
                                   UnixSocketProtocol::ServerSide *proto,
                                   std::function<void(int)> on_client_error,
                                   int max_pending_cnx) :
    path_(path),
    max_pending_cnx_(max_pending_cnx),
    proto_(proto),
    on_client_error_(on_client_error){
  if (!socket_)  // server not valid if socket is not valid
    return;
  if (nullptr == proto)  // server not valid without protocol
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
  //printf("%d, %s\n", socket_.fd_, path_.c_str());
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
  done_ = std::async(std::launch::async,
                     [](UnixSocketServer *self){self->client_interaction();},
                     this);
}

UnixSocketServer::~UnixSocketServer() {
  if (done_.valid()) {
    quit_.store(1);
    done_.get();
    unlink (path_.c_str());
  }
}

short UnixSocketServer::notify_update() {
  std::unique_lock<std::mutex> lock(clients_mutex_);
  clients_notified_.clear();
  // re-sending connect message
  //auto msg = proto_->get_connect_msg_();
  for (auto &it: clients_){
    //auto res = send(it, &msg, sizeof(msg), MSG_NOSIGNAL);
    auto res = send(it, &proto_->update_msg_, sizeof(proto_->update_msg_), MSG_NOSIGNAL);
    if (-1 == res)
      std::cout << "ERROR send (update)" << std::endl;
    else
      clients_notified_.insert(it);
  }
  return clients_notified_.size();
}

bool UnixSocketServer::is_valid() const {
  return is_binded_ && is_listening_;
}

void UnixSocketServer::client_interaction() {
  fd_set allset;
  FD_ZERO(&allset);
  FD_SET(socket_.fd_, &allset);
  auto maxfd = socket_.fd_;
  struct timeval tv;  // select timeout
  auto cnx_msg = proto_->get_connect_msg_();
  char	buf[8192];  // FIXME MAXLINE
  std::vector<int> clients_to_remove;
  while (0 == quit_.load()) {
    // reset timeout since select may change values
    tv.tv_sec = 0;
    tv.tv_usec = 10000;  // 10 msec
    auto rset = allset;  /* rset gets modified each time around */
    if (select(maxfd + 1, &rset, NULL, NULL, &tv) < 0) {
      perror("select error");
      continue;
    }
    std::unique_lock<std::mutex> lock(clients_mutex_);
    if (FD_ISSET(socket_.fd_, &rset)) {
      // accept new client request
      auto clifd = accept(socket_.fd_, NULL, NULL);
      if (clifd < 0)
        perror("accept");
      FD_SET(clifd, &allset);
      if (clifd > maxfd)
        maxfd = clifd;  // max fd for select()
      pending_clients_.insert(clifd);
      auto res = send(clifd, &cnx_msg, sizeof(cnx_msg), MSG_NOSIGNAL);
      if (-1 == res)
        perror("send");
      continue;
    }
    // checking disconnection
    for (auto &it : clients_) {
      if (FD_ISSET(it, &rset)) {
        auto nread = read(it, buf, 8192);  // MAXLINE
        if (nread < 0) {
          perror("read disconnection");
          clients_to_remove.push_back(it);
          if (clients_notified_.end() != clients_notified_.find(it)) {
            on_client_error_(it);
          }
        } else if (nread == 0) {
          std::printf("(server) closed: fd %d\n", it);
          FD_CLR(it, &allset);
          close(it);
        } else {
          // send quit ack
          auto res = send(it, &proto_->quit_msg_, sizeof(proto_->quit_msg_), MSG_NOSIGNAL);
          if (-1 == res)
            perror("send (ack quit)");
          if (proto_->on_disconnect_cb_)
            proto_->on_disconnect_cb_(it);
          clients_to_remove.push_back(it);
          std::cout << "client quit" << std::endl;
        }
      }
    }
    // cleaning clients_ if necessary
    for (auto &it : clients_to_remove) {
      auto cli = std::find(clients_.begin(), clients_.end(), it);
      clients_.erase(cli);
    }
    clients_to_remove.clear();
    // checking ack from clients
    for (auto &it : pending_clients_) {
      if (FD_ISSET(it, &rset)) {
        auto nread = read(it, buf, 8192);  // MAXLINE
        if (nread < 0) {
          perror("read ack");
          clients_to_remove.push_back(it);
        } else if (nread == 0) {
          std::printf("bug checking connection ack from client");
          clients_to_remove.push_back(it);
          FD_CLR(it, &allset);
          close(it);
        } else { 
          clients_.push_back(it);
          clients_to_remove.push_back(it);
          if (proto_->on_connect_cb_)
            proto_->on_connect_cb_(it);
          std::printf("(server) new connection: fd %d\n", it);
        }
      }
    }
    // removing pending client if necessary
    for (auto &it : clients_to_remove) {
      pending_clients_.erase(it);
    }
    clients_to_remove.clear();
  }  // while (!quit_)
}

}  // namespace shmdata
