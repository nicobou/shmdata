/*
 *
 * posture is free software; you can redistribute it and/or
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

#include "./posture_display.hpp"

#include <iostream>

using namespace std;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureDisplay,
                                     "pcldisplay",
                                     "Point Clouds Display",
                                     "video",
                                     "reader",
                                     "Display point clouds in a window",
                                     "LGPL",
                                     "Emmanuel Durand");

PostureDisplay::~PostureDisplay() { disconnect_all(); }

PostureDisplay::PostureDisplay(QuiddityConfiguration&&) : shmcntr_(static_cast<Quiddity*>(this)) {
  shmcntr_.install_connect_method([this](const string path) { return connect(path); },
                                  [this](const string path) { return disconnect(path); },
                                  [this]() { return disconnect_all(); },
                                  [this](const string caps) { return can_sink_caps(caps); },
                                  1);
}

bool PostureDisplay::connect(std::string shmdata_socket_path) {
  if (display_ != nullptr) return false;

  display_ = std::make_unique<Display>(shmdata_socket_path);

  reader_ = std::make_unique<ShmdataFollower>(
      this,
      shmdata_socket_path,
      [=](void* data, size_t size) {
        if (!display_mutex_.try_lock()) return;

        if (display_ == nullptr) {
          display_mutex_.unlock();
          return;
        }

        if (reader_caps_ == string(POINTCLOUD_TYPE_COMPRESSED) ||
            reader_caps_ == string(POINTCLOUD_TYPE_BASE)) {
          vector<char> buffer((char*)data, (char*)data + size);
          display_->setInputCloud(buffer, reader_caps_ == string(POINTCLOUD_TYPE_COMPRESSED));
        } else if (reader_caps_ == string(POLYGONMESH_TYPE_BASE)) {
          vector<unsigned char> buffer((unsigned char*)data, (unsigned char*)data + size);
          display_->setPolygonMesh(buffer);
        }

        display_mutex_.unlock();
      },
      [=](string caps) {
        unique_lock<mutex> lock(display_mutex_);
        reader_caps_ = caps;
      });

  return true;
}

bool PostureDisplay::disconnect(std::string /*unused*/) { return disconnect_all(); }

bool PostureDisplay::disconnect_all() {
  std::lock_guard<mutex> lock(display_mutex_);
  reader_.reset();
  display_.reset();
  return true;
}

bool PostureDisplay::can_sink_caps(std::string caps) {
  return (caps == POINTCLOUD_TYPE_BASE) || (caps == POINTCLOUD_TYPE_COMPRESSED) ||
         (caps == POLYGONMESH_TYPE_BASE);
}

}  // namespace switcher
