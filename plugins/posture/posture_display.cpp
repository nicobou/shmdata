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
using namespace switcher::data;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureDisplay,
                                     "Point Clouds Display",
                                     "video",
                                     "Display point clouds in a window",
                                     "LGPL",
                                     "pcldisplaysink",
                                     "Emmanuel Durand");

PostureDisplay::PostureDisplay(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper> ()) {
}

PostureDisplay::~PostureDisplay() {
}

bool
PostureDisplay::init() {
  init_segment(this);

  install_connect_method(std::bind(&PostureDisplay::connect, this, std::placeholders::_1),
                         std::bind(&PostureDisplay::disconnect, this, std::placeholders::_1),
                         std::bind(&PostureDisplay::disconnect_all, this),
                         std::bind(&PostureDisplay::can_sink_caps, this, std::placeholders::_1),
                         1);

  return true;
}

bool
PostureDisplay::connect(std::string shmdata_socket_path) {
  if (display_ != nullptr)
    return false;

  display_ = make_shared<Display> (shmdata_socket_path);

  ShmdataAnyReader::ptr reader = make_shared<ShmdataAnyReader> ();
  reader->set_path(shmdata_socket_path);

  // This is the callback for when new clouds are received
  reader->set_callback([=] (void *data, int size, unsigned long long /* unused */, const char *type, void * /*unused */ )
  {
    if (!display_mutex_.try_lock())
      return;

    if (display_ == nullptr)
    {
      display_mutex_.unlock();
      return;
    }

    if (string(type) == string(POINTCLOUD_TYPE_COMPRESSED) || string(type) == string(POINTCLOUD_TYPE_BASE))
    {
      vector<char> buffer((char *)data, (char *)data + size);
      display_->setInputCloud(buffer, string(type) == string(POINTCLOUD_TYPE_COMPRESSED));
    }
    else if (string(type) == string(POLYGONMESH_TYPE_BASE))
    {
        vector<unsigned char> buffer((unsigned char*)data, (unsigned char*)data + size);
        display_->setPolygonMesh(buffer);
    }

    display_mutex_.unlock();
  }, nullptr);

  reader->start();
  register_shmdata(reader);

  return true;
}

bool
PostureDisplay::disconnect(std::string /*unused*/) {
  return disconnect_all();
}

bool
PostureDisplay::disconnect_all() {
  std::lock_guard<mutex> lock(display_mutex_);
  clear_shmdatas();
  display_.reset();
  return true;
}

bool
PostureDisplay::can_sink_caps(std::string caps) {
  return (caps == POINTCLOUD_TYPE_BASE)
      || (caps == POINTCLOUD_TYPE_COMPRESSED)
      || (caps == POLYGONMESH_TYPE_BASE);
}
}  // namespace switcher
