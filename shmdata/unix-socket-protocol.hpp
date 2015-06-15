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
#include <array>

namespace shmdata{
namespace UnixSocketProtocol{

struct onConnectData {
  onConnectData(size_t shm_size,
                const std::string &user_data);
  onConnectData() = default;
  // data to distribute by server at connection
  const unsigned short msg_type_{0}; 
  size_t shm_size_{0};
  std::array<char, 4096> user_data_{{}};  
};

struct UpdateMsg {
  const unsigned short msg_type_{1}; 
  size_t size_{0};
};

struct QuitMsg {
  const unsigned short msg_type_{2}; 
};


// Server -----------------------------------------------------
struct ServerSide {
  using onClientConnect = std::function<void(int id)>;
  using onClientDisconnect = std::function<void(int id)>;
  onClientConnect on_connect_cb_;
  onClientDisconnect on_disconnect_cb_;
  // (server) get buffers to send back to clients when connecting
  using MsgOnConnect = std::function<onConnectData()>;
  MsgOnConnect get_connect_msg_;
  UpdateMsg update_msg_{};
  QuitMsg quit_msg_{};
  ServerSide(onClientConnect occ,
             onClientDisconnect ocd,
             MsgOnConnect gocm) :
      on_connect_cb_(occ),
      on_disconnect_cb_(ocd),
      get_connect_msg_(gocm) {
  }
};

// Client -----------------------------------------------------

struct ClientSide {
  using onServerConnected = std::function<void()>;
  using onServerDisconnected = std::function<void()>;
  using onUpdate = std::function<void(size_t)>;  // the size that has been writen
  onServerConnected on_connect_cb_{};
  onServerDisconnected on_disconnect_cb_{};
  onConnectData data_{};
  onUpdate on_update_cb_{};
  UpdateMsg update_msg_{};
  QuitMsg quit_msg_{};
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
