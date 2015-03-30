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


#ifndef _SHMDATA_UNIX_SOCKET_PROTOCOL_H_
#define _SHMDATA_UNIX_SOCKET_PROTOCOL_H_

#include <sys/types.h>
#include <sys/socket.h>
#include <string>
#include <functional>

namespace shmdata{
namespace UnixSocketProtocol{

using onPeerConnect = std::function<void(int id)>;
using onPeerDisconnect = std::function<void(int id)>;
using socketMsg_t = struct socketMsg {
socketMsg(const struct iovec *iovec, size_t iovec_len):
    iov_(iovec), iov_len_(iovec_len){}
const struct iovec *iov_{nullptr};
size_t iov_len_{0};
};

struct onConnectData {
  onConnectData(size_t shm_size,
                key_t key,
                const std::string &user_data);
  // data to distribute at connection
  size_t shm_size_;
  key_t shm_key_;
  std::string user_data_;  
};

// Server -----------------------------------------------------
// a basic initialisation and life monitoring protocol
struct ServerSide {
// connect/disconnect callbacks
onPeerConnect on_connect_cb_{};
onPeerDisconnect on_disconnect_cb_{};
// (server) get buffers to send back to clients when connecting
using iovServOnConnect = std::function<socketMsg_t()>;
iovServOnConnect get_connect_iov_{};
};

// constructing onConnectData
struct onConnectDataMaker : public onConnectData {
  // socket data structure pointing to members initialized in ctor
  size_t iovec_len_{3};
  const struct iovec iovec_[3];
  // ctor
  onConnectDataMaker(size_t shm_size,
                key_t key,
                const std::string &user_data);
  onConnectDataMaker() = delete;
  socketMsg_t get_connect_iov();
};

// client -------------------------------------------------------


}  // namespace UnixSocketProtocol
}  // namespace shmdata
#endif
