/*
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

#include "./follower.hpp"
#include <thread>
#include <utility>
#include "./file-monitor.hpp"
#include "./unix-socket-server.hpp"

namespace shmdata {

Follower::Follower(const std::string& path,
                   Reader::onData cb,
                   Reader::onServerConnected osc,
                   Reader::onServerDisconnected osd,
                   AbstractLogger* log)
    : log_(log),
      path_(path),
      on_data_cb_(cb),
      osc_(osc),
      osd_(osd),
      reader_(fileMonitor::is_unix_socket(path_, log_)
                  ? new Reader(path_, on_data_cb_, osc_, [&]() { on_server_disconnected(); }, log_)
                  : nullptr) {
  if (!reader_ || !(*reader_.get()))
    monitor_ = std::async(std::launch::async, [this]() { monitor(); });
}

Follower::~Follower() {
  is_destructing_ = true;
  quit_.store(true);
  if (monitor_.valid()) monitor_.get();
}

void Follower::monitor() {
  auto do_sleep = true;
  // give the change the reader to fail twice before cleaning dead shmdata:
  // auto successive_fail = 0;
  while (!quit_.load()) {
    if (fileMonitor::is_unix_socket(path_, log_)) {
      do_sleep = false;
      // log_->debug("file detected, creating reader");
      reader_.reset(
          new Reader(path_, on_data_cb_, osc_, [&]() { on_server_disconnected(); }, log_));
      if (*reader_.get()) {
        quit_.store(true);
      } else {
        reader_.reset();
        // log_->debug("file % exists but reader failed", path_);
        // if (1 == successive_fail) {
        //   if(!force_sockserv_cleaning(path_, log_))
        //     log_->warning("follower shmpath is not dead shmdata that can be
        //     cleaned");
        //   else
        //     log_->debug("shmdata follower detected and cleaned a possible
        //     dead shmdata: %",
        //                 path_);
        //   successive_fail = 0;
        // } else { ++successive_fail; }
      }
    }
    if (do_sleep) std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }  // end while
  quit_.store(true);
}

void Follower::on_server_disconnected() {
  log_->debug("follower %", __FUNCTION__);
  // starting monitor
  if (!is_destructing_) {
    quit_.store(false);
    monitor_ = std::async(std::launch::async, [this]() { monitor(); });
  }
  // calling user callback
  if (osd_) osd_();
}

}  // namespace shmdata
