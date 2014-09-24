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

#include "./posture_colorize.hpp"

#include <iostream>
#include <thread>

using namespace std;
using namespace switcher::data;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureColorize,
                                     "Project texture onto mesh",
                                     "video",
                                     "Project texture onto mesh, based on calibration",
                                     "LGPL",
                                     "texturetomeshsink", "Emmanuel Durand");

PostureColorize::PostureColorize():
    custom_props_(std::make_shared<CustomPropertyHelper> ()) {
}

PostureColorize::~PostureColorize() {
  stop();
}

bool
PostureColorize::start() {

  return true;
}

bool
PostureColorize::stop() {
  lock_guard<mutex> lock(mutex_);

  return true;
}

bool
PostureColorize::init() {
  init_startable(this);
  init_segment(this);

  install_connect_method(std::bind(&PostureColorize::connect, this, std::placeholders::_1),
                         std::bind(&PostureColorize::disconnect, this, std::placeholders::_1),
                         std::bind(&PostureColorize::disconnect_all, this),
                         std::bind(&PostureColorize::can_sink_caps, this, std::placeholders::_1),
                         1);

  return true;
}

bool
PostureColorize::connect(std::string /*unused*/) {
  return true;
}

bool
PostureColorize::disconnect(std::string /*unused*/) {
  return true;
}

bool
PostureColorize::disconnect_all() {
  return true;
}

bool
PostureColorize::can_sink_caps(std::string caps) {
  return (caps == POINTCLOUD_TYPE_BASE)
      || (caps == POINTCLOUD_TYPE_COMPRESSED);
}

}  // namespace switcher
