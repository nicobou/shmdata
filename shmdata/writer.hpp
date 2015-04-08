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

#ifndef _SHMDATA_WRITER_H_
#define _SHMDATA_WRITER_H_

#include <string>
#include "shmdata/unix-socket-server.hpp"
#include "shmdata/unix-socket-protocol.hpp"
#include "shmdata/sysv-shm.hpp"
#include "shmdata/sysv-sem.hpp"
#include "./safe-bool-idiom.hpp"

namespace shmdata{

class writer: public SafeBoolIdiom {
 public:
  writer(const std::string &path, size_t memsize, const std::string &data_descr);
  //~writer();
  writer() = delete;
  writer(const writer &) = delete;
  writer& operator=(const writer&) = delete;
  writer& operator=(writer&&) = default;

  bool copy_to_shm(void *data, size_t size);
  
 private:
  std::string path_; 
  UnixSocketProtocol::onConnectDataMaker connect_data_;
  UnixSocketProtocol::ServerSide proto_;
  UnixSocketServer srv_;
  sysVShm shm_;
  sysVSem sem_;
  bool is_valid_{true};
  bool is_valid() const final{return is_valid_;}
};


}  // namespace shmdata
#endif
