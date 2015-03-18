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


#ifndef _SHMDATA_UNIX_SOCKET_H_
#define _SHMDATA_UNIX_SOCKET_H_

#include "./safe-bool-idiom.hpp"

namespace shmdata{

class UnixSocket: public SafeBoolIdiom {
 public:
  UnixSocket(const std::string &path);
  ~UnixSocket();
  UnixSocket() = delete;
  
  private:
  std::string path_;
  int socket_{-1};
  bool is_valid() const final;
};

}  // namespace shmdata
#endif
