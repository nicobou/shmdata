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

#include "./shmdata-follower.hpp"

namespace switcher {
ShmdataFollower::ShmdataFollower(const std::string &path,
                                 shmdata::Reader::onData od,
                                 shmdata::Reader::onServerConnected osc,
                                 shmdata::Reader::onServerDisconnected osd):
    path_(path),
    od_(od),
    osc_(osc),
    osd_(osd),
    follower_(path_,
              [this](void *data, size_t size){this->on_data(data, size);},
              osc_,
              osd_,
              &logger_),
    json_description_(new JSONBuilder()) {
  make_json_description();
}


void ShmdataFollower::make_json_description() {
  json_description_->reset();
  json_description_->begin_object();
  json_description_->add_string_member("path", path_.c_str());
  json_description_->end_object();
}

JSONBuilder::Node ShmdataFollower::get_json_root_node() {
  return json_description_->get_root();
}

void
ShmdataFollower::on_data(void *data, size_t size) {
  if (!od_) {
    g_warning("data not handled by follower");
    return;
  }
  od_(data, size);
}

}
