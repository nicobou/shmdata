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

#ifndef _SHMDATA_READER_H_
#define _SHMDATA_READER_H_

#include <string>
#include <future>
#include "shmdata/sysv-shm.hpp"
#include "shmdata/sysv-sem.hpp"
#include "shmdata/unix-socket-client.hpp"
#include "./safe-bool-idiom.hpp"
#include "./abstract-logger.hpp"

namespace shmdata{
class Reader: public SafeBoolIdiom {
 public:
  using on_data_cb = std::function<void(void *, size_t)>;
  Reader(const std::string &path, on_data_cb cb, AbstractLogger *log);
  ~Reader();
  Reader() = delete;
  Reader(const Reader &) = delete;
  Reader& operator=(const Reader&) = delete;
  Reader& operator=(Reader&&) = default;

 private:
  AbstractLogger *log_;
  std::string path_;
  on_data_cb on_data_cb_;
  sysVShm shm_;
  sysVSem sem_;
  UnixSocketProtocol::ClientSide proto_;
  UnixSocketClient cli_;
  bool do_read_{true};
  bool is_valid_{true};
  bool is_valid() const final{return is_valid_;}
  void on_server_connected();
  void on_server_disconnected();
  bool on_buffer(sysVSem *sem, size_t size);
};

}  // namespace shmdata
#endif
