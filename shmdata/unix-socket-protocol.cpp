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

#include <limits> 
#include "./unix-socket-protocol.hpp"

namespace shmdata{
namespace UnixSocketProtocol{

onConnectData::onConnectData(size_t shm_size,
                             const std::string &user_data) :
    shm_size_(shm_size) {
  auto size = user_data.size();
  std::copy(user_data.begin(), user_data.end(), user_data_.begin());
  user_data_[size] = '\0';
}
    
}  // namespace UnixSocketProtocol
}  // namespace shmdata
