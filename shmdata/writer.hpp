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
#include <memory>
#include "shmdata/unix-socket-server.hpp"
#include "shmdata/unix-socket-protocol.hpp"
#include "shmdata/sysv-shm.hpp"
#include "shmdata/sysv-sem.hpp"
#include "./safe-bool-idiom.hpp"
#include "./abstract-logger.hpp"

namespace shmdata{
class OneWriteAccess;
class Writer: public SafeBoolIdiom {
 public:
  Writer(const std::string &path,
         size_t memsize,
         const std::string &data_descr,
         AbstractLogger *log,
         UnixSocketProtocol::ServerSide::onClientConnect on_client_connect = nullptr,
         UnixSocketProtocol::ServerSide::onClientDisconnect on_client_disconnect = nullptr);
  ~Writer() = default;
  Writer() = delete;
  Writer(const Writer &) = delete;
  Writer& operator=(const Writer&) = delete;
  Writer& operator=(Writer&&) = default;

  // copy to shmdata
  bool copy_to_shm(void *data, size_t size);
  // direct access to the memory with lock
  std::unique_ptr<OneWriteAccess> get_one_write_access(size_t size = 0);
  // (for C wrappers) direct access without uniqueptr (user need to delete)
  OneWriteAccess *get_one_write_access_ptr(size_t size = 0);
 private:
  std::string path_; 
  UnixSocketProtocol::onConnectData connect_data_;
  UnixSocketProtocol::ServerSide proto_;
  UnixSocketServer srv_;
  sysVShm shm_;
  sysVSem sem_;
  AbstractLogger *log_;
  bool is_valid_{true};
  bool is_valid() const final{return is_valid_;}
};

// see check-shmdata
class OneWriteAccess {
  friend Writer;
 public:
  void *get_mem() {return mem_;};
  short notify_clients(); // must be called once, return number of clients notified
 private:
  OneWriteAccess(sysVSem *sem,
                 void *mem,
                 UnixSocketServer *srv,
                 size_t size,
                 AbstractLogger *log);
  OneWriteAccess& operator=(OneWriteAccess&&) = default;
  WriteLock wlock_;
  void *mem_;
  UnixSocketServer *srv_;
  size_t size_;
  AbstractLogger *log_;
  bool has_notified_{false};
};

}  // namespace shmdata
#endif
