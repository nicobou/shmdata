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

#include <thread>
#include <utility>
#include "./follower.hpp"
#include "./file-monitor.hpp"

namespace shmdata{

Follower::Follower(const std::string &path,
                   Reader::onData cb,
                   Reader::onServerConnected osc,
                   Reader::onServerDisconnected osd,
                   AbstractLogger *log) :
    log_(log),
    path_(path),
    on_data_cb_(cb),
    osc_(osc),
    osd_(osd),
    reader_(new Reader(path_, on_data_cb_, osc_, [&](){on_server_disconnected();}, log_)){
  if (!(*reader_.get())){
    monitor_ = std::async(std::launch::async, [this](){monitor();});
  }
}

Follower::~Follower(){
  quit_.store(true);
  if (monitor_.valid()) 
    monitor_.get();
}

void Follower::monitor(){
  auto do_sleep = true;
  while (!quit_.load()) {
    if (fileMonitor::is_unix_socket(path_, log_)) {
      do_sleep = false;
      reader_.reset(new Reader(path_, on_data_cb_, osc_, [&](){on_server_disconnected();}, log_));
      if (*reader_.get()) {
        quit_.store(true);
      } else {
        log_->error("file % exists but reader failed", path_);
      }
    } 
    if (do_sleep)
      std::this_thread::sleep_for(std::chrono::milliseconds (30));
  }  // end while
}

void Follower::on_server_disconnected() {
  log_->debug("follower %", __FUNCTION__);
  // starting monitor
  quit_.store(false);
  monitor_ = std::async(std::launch::async, [this](){monitor();});
  // calling user callback
  osd_();
}

}  // namespace shmdata
