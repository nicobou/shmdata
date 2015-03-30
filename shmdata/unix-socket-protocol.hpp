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
  onConnectData() = default;
  // data to distribute by server at connection
  size_t shm_size_{0};
  key_t shm_key_{0};
  std::string user_data_{};  
};

// Server -----------------------------------------------------
struct ServerSide {
  using onClientConnect = std::function<void(int id)>;
  using onClientDisconnect = std::function<void(int id)>;
  onClientConnect on_connect_cb_{};
  onClientDisconnect on_disconnect_cb_{};
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

// Client -----------------------------------------------------
// placeholder for receiving connection data
template<size_t _size>
struct onConnectDataReceiver : public onConnectData {
  onConnectDataReceiver() :
      iovec_{
  {&shm_size_, sizeof(size_t)},
  {&shm_key_, sizeof(key_t)},
  {user_data, _size}} {
  }
  std::string get_user_data(){
    return std::string(static_cast<char *>(iovec_[2].iov_base),
                       0,
                       iovec_[2].iov_len);
}
  char user_data[_size];
  size_t iovec_len_{3};
  const struct iovec iovec_[3];
};

struct ClientSide {
  using onServerConnected = std::function<void(int id)>;
  using onServerDisconnected = std::function<void(int id)>;
  onServerConnected on_connect_cb_{};
  onServerDisconnected on_disconnect_cb_{};
  onConnectDataReceiver<4096> data_{};
  //  void set_user_data(std::string &&str);
};


}  // namespace UnixSocketProtocol
}  // namespace shmdata
#endif
