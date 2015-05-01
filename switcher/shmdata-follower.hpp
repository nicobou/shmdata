/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef __SWITCHER_SHMDATA_FOLLOWER_H__
#define __SWITCHER_SHMDATA_FOLLOWER_H__

#include <functional>
#include <memory>
#include <string>
#include <shmdata/follower.hpp>
#include "./json-builder.hpp"
#include "./shmdata-glib-logger.hpp"

namespace switcher {
class ShmdataFollower {
 public:
  typedef std::shared_ptr<ShmdataFollower> ptr;
  ShmdataFollower(const std::string &path,
                  shmdata::Reader::onData cb,
                  shmdata::Reader::onServerConnected osc,
                  shmdata::Reader::onServerDisconnected osd);
  ~ShmdataFollower() = default;
  ShmdataFollower(const ShmdataFollower &) = delete;
  ShmdataFollower &operator=(const ShmdataFollower &) = delete;

  std::string get_path() const;
  JSONBuilder::Node get_json_root_node();

 private:
  ShmdataGlibLogger logger_{};
  std::string path_;
  shmdata::Reader::onData od_;
  shmdata::Reader::onServerConnected osc_;
  shmdata::Reader::onServerDisconnected osd_;
  shmdata::Follower follower_;
  JSONBuilder::ptr json_description_;

  void on_data(void *data, size_t data_size);
  void make_json_description();
};

}  // namespace switcher
#endif
