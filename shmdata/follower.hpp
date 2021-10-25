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

#ifndef _SHMDATA_FOLLOWER_H_
#define _SHMDATA_FOLLOWER_H_

#include <atomic>
#include <future>
#include <string>
#include "./abstract-logger.hpp"
#include "./reader.hpp"

namespace shmdata {

class Follower {
 public:
  /**
   * \brief Construct a Follower object that read a shmdata, and handle connection/disconnection
   * of the writer. Information and data are provided asynchronously by the Follower.
   * though callbacks.
   *
   * \param   path Shmdata path to follow.
   * \param   cb   Callback to be triggered when a frame is published.
   * \param   osc  Callback to be triggered when the follower connects with the shmdata writer.
   * \param   osd  Callback to be triggered when the follower disconnects from the shmdata writer.
   * \param   log  Log object where to write internal logs.
   *
   */
  Follower(const std::string& path,
           Reader::onData cb,
           Reader::onServerConnected osc,
           Reader::onServerDisconnected osd,
           AbstractLogger* log);

  /**
   * \brief Destruct the follower and release resources acquired.
   *
   */
  ~Follower();
  Follower() = delete;
  Follower(const Follower&) = delete;
  Follower& operator=(const Follower&) = delete;
  Follower& operator=(Follower&&) = delete;

 private:
  bool is_destructing_{false};
  AbstractLogger* log_;
  std::string path_;
  Reader::onData on_data_cb_;
  Reader::onServerConnected osc_;
  Reader::onServerDisconnected osd_;
  std::future<void> monitor_{};
  std::atomic<bool> quit_{false};
  std::unique_ptr<Reader> reader_;
  void monitor();
  void on_server_disconnected();
};

}  // namespace shmdata
#endif
