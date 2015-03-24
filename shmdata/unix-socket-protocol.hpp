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
#include <functional>

namespace shmdata{

// a basic initialisation and life monitoring protocol
struct UnixSocketProtocol {
  // connect/disconnect callbacks
  using onPeerConnect = std::function<void(int id)>;
  onPeerConnect on_connect_cb_{};
  using onPeerDisconnect = std::function<void(int id)>;
  onPeerDisconnect on_disconnect_cb_{};
  // (server) get buffers to send back to clients when conecting
  using iovServOnConnect = std::function<std::pair<const struct iovec *, size_t>()>;
  iovServOnConnect get_connect_iov_{};
  };

struct onConnectData {
  // data to distribute at connection
  size_t shm_size_;
  std::string shm_path_;
  std::string user_data_;
  // socket data structure pointing to previous members
  const struct iovec iovec_[3];
  // ctor
  onConnectData(size_t shm_size,
                const std::string &shm_path,
                const std::string &user_data);
  onConnectData() = delete;
};

}  // namespace shmdata
#endif
