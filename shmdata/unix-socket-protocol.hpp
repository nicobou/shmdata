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


// FIXME get rid of update...
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
                const std::string &user_data);
  onConnectData() = default;
  // data to distribute by server at connection
  size_t shm_size_{0};
  std::string user_data_{};  
};

struct onUpdateData {
  size_t count_{0};
};

// Server -----------------------------------------------------
// constructing onConnectData
struct onConnectDataMaker : public onConnectData {
  // socket data structure pointing to members initialized in ctor
  size_t iovec_len_{2};
  const struct iovec iovec_[2];
  // ctor
  onConnectDataMaker(size_t shm_size,
                     const std::string &user_data);
  onConnectDataMaker() = delete;
  socketMsg_t get_connect_iov();
};

// constructing onUpdateData
struct onUpdateDataMaker : public onUpdateData {
  // socket data structure pointing to members initialized in ctor
  size_t iovec_len_{1};
  const struct iovec iovec_[1];
  // ctor
  onUpdateDataMaker();
  socketMsg_t get_update_iov();
};

struct ServerSide {
  using onClientConnect = std::function<void(int id)>;
  using onClientDisconnect = std::function<void(int id)>;
  onClientConnect on_connect_cb_{};
  onClientDisconnect on_disconnect_cb_{};
  // (server) get buffers to send back to clients when connecting
  using iovServOnConnect = std::function<socketMsg_t()>;
  iovServOnConnect get_connect_iov_{};
  onUpdateDataMaker updater_{};
  socketMsg_t get_update_iov(){return updater_.get_update_iov();}
  ServerSide(onClientConnect occ,
             onClientDisconnect ocd,
             iovServOnConnect isoc) :
      on_connect_cb_(occ),
      on_disconnect_cb_(ocd),
      get_connect_iov_(isoc) {
  }
};

// Client -----------------------------------------------------
// placeholder for receiving connection data
template<size_t _size>
struct onConnectDataReceiver : public onConnectData {
  onConnectDataReceiver() :
      iovec_{
  {&shm_size_, sizeof(size_t)},
  {user_data_, _size}} {
  }
  std::string get_user_data(){
    return std::string(static_cast<char *>(iovec_[1].iov_base),
                       0,
                       iovec_[1].iov_len);
}
  char user_data_[_size];
  size_t iovec_len_{2};
  const struct iovec iovec_[2];
};

struct ClientSide {
  using onServerConnected = std::function<void(int id)>;  // FIXME remove id
  using onServerDisconnected = std::function<void(int id)>;
  using onUpdate = std::function<void(int id)>;
  onServerConnected on_connect_cb_{};
  onServerDisconnected on_disconnect_cb_{};
  onConnectDataReceiver<65536> data_{};
  onUpdate on_update_cb_{};
  ClientSide(onServerConnected osc,
             onServerDisconnected osd,
             onUpdate ou) :
      on_connect_cb_(osc),
      on_disconnect_cb_(osd),
      on_update_cb_(ou) {
  }
};


}  // namespace UnixSocketProtocol
}  // namespace shmdata
#endif
