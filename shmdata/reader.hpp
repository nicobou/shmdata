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
#include <memory>
#include "shmdata/sysv-shm.hpp"
#include "shmdata/sysv-sem.hpp"
#include "shmdata/unix-socket-client.hpp"
#include "./safe-bool-idiom.hpp"
#include "./abstract-logger.hpp"

namespace shmdata{
class Reader: public SafeBoolIdiom {
 public:
  using onData = std::function<void(void *, size_t)>;
  using onServerConnected = std::function<void(const std::string &)>;
  using onServerDisconnected = std::function<void()>;
  Reader(const std::string &path,
         onData cb,
         onServerConnected osc,
         onServerDisconnected osd,
         AbstractLogger *log);
  ~Reader() = default;
  Reader() = delete;
  Reader(const Reader &) = delete;
  Reader& operator=(const Reader&) = delete;
  Reader& operator=(Reader&&) = default;

 private:
  AbstractLogger *log_;
  std::string path_;
  onData on_data_cb_;
  onServerConnected on_server_connected_cb_;
  onServerDisconnected on_server_disconnected_cb_;
  std::unique_ptr<sysVShm> shm_{nullptr};
  std::unique_ptr<sysVSem> sem_{nullptr};
  UnixSocketProtocol::ClientSide proto_;
  std::unique_ptr<UnixSocketClient> cli_;
  bool is_valid_{false};
  bool is_valid() const final{return is_valid_;}
  void on_server_connected();
  void on_server_disconnected();
  bool on_buffer(sysVSem *sem, size_t size);
};

}  // namespace shmdata
#endif
