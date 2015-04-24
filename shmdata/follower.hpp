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

#ifndef _SHMDATA_FOLLOWER_H_
#define _SHMDATA_FOLLOWER_H_

#include <string>
#include <future>
#include <atomic>
#include "./reader.hpp"
#include "./abstract-logger.hpp"

namespace shmdata{
class Follower {
 public:
  Follower(const std::string &path,
           Reader::onData cb,
           Reader::onServerConnected osc,
           Reader::onServerDisconnected osd,
           AbstractLogger *log);
  ~Follower();
  Follower() = delete;
  Follower(const Follower &) = delete;
  Follower& operator=(const Follower&) = delete;
  Follower& operator=(Follower&&) = delete;

 private:
  bool is_destructing_{false};
  AbstractLogger *log_;
  std::string path_;
  Reader::onData on_data_cb_;
  Reader::onServerConnected osc_;
  Reader::onServerDisconnected osd_;
  std::future<void> monitor_{};
  std::unique_ptr<Reader> reader_;
  std::atomic<bool> quit_{false};
  void monitor();
  void on_server_disconnected();
};

}  // namespace shmdata
#endif
