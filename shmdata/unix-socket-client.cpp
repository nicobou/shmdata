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
#include <sys/uio.h>
#include <errno.h>
#include <string.h>
#include "./unix-socket-client.hpp"

// OSX compatibility
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL SO_NOSIGPIPE
#endif

namespace shmdata{

UnixSocketClient::UnixSocketClient(const std::string &path,
                                   UnixSocketProtocol::ClientSide *proto,
                                   AbstractLogger *log) :
    path_(path),
    socket_(log),
    log_(log),
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
    int err = errno;
    log_->error("connect %", strerror(err));
    return;
  }
  is_valid_ = true;
  done_ = std::async(std::launch::async,
                     [](UnixSocketClient *self){self->server_interaction();},
                     this);
}

UnixSocketClient::~UnixSocketClient() {
  if (done_.valid()) {
    if (is_valid_) {
      auto res = send(socket_.fd_, &proto_->quit_msg_, sizeof(proto_->quit_msg_), MSG_NOSIGNAL);
      if (-1 == res)
        log_->error("send (client trying to quit)");
    }
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
  bool quit = false;
  if (0 != quit_.load())
    quit = true;
  bool quit_acked = false;
  while (!quit && ! quit_acked) {
    // reset timeout since select may change values
    tv.tv_sec = 0;
    tv.tv_usec = 10000;  // 10 msec
    auto rset = allset;  /* rset gets modified each time around */
    if (select(maxfd + 1, &rset, NULL, NULL, &tv) < 0) {
      int err = errno;
      log_->error("select %", strerror(err));
      continue;
    }
    if (FD_ISSET(socket_.fd_, &rset)) {
      ssize_t nread;
      if (!connected_) 
        nread = read(socket_.fd_, &proto_->data_, sizeof(UnixSocketProtocol::onConnectData));
      else
        nread = read(socket_.fd_, &proto_->update_msg_, sizeof(proto_->update_msg_));
      if (nread < 0) {
        int err = errno;
        log_->error("read %", strerror(err));
      } else if (nread == 0) {
        log_->debug("socket client, server disconnected");
        proto_->on_disconnect_cb_();
        // disable socket
        FD_CLR(socket_.fd_, &allset);
        if (0 != close(socket_.fd_)){
          int err = errno;
          log_->error("client closing socket %", strerror(err));
        }
        socket_.fd_ = -1; is_valid_ = false; quit = true; quit_acked = true;
      } else { /* process server′s message */
        if (!connected_) {
          // ack connection
          auto res = send(socket_.fd_,
                          &proto_->data_,
                          sizeof(UnixSocketProtocol::onConnectData),
                          MSG_NOSIGNAL);
          if (-1 == res) {
            int err = errno;
            log_->error("client sending ack %", strerror(err));
          }
          proto_->on_connect_cb_();
          connected_ = true;
        } else {
          if (1 == proto_->update_msg_.msg_type_)
            proto_->on_update_cb_(proto_->update_msg_.size_);
          else if ((2 == proto_->update_msg_.msg_type_)) {
            proto_->on_disconnect_cb_();
            // disable socket
            FD_CLR(socket_.fd_, &allset);
            if (0 != close(socket_.fd_)){
              int err = errno;
              log_->error("client closing socket %", strerror(err));
            }
            socket_.fd_ = -1; is_valid_ = false; quit = true; quit_acked = true;
          }
          else
            quit_acked = true;
        }
      }
    }
    if (0 != quit_.load())
      quit = true;
  }  // while (!quit_)
}

}  // namespace shmdata
