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

#include "switcher/std2.hpp"
#include "switcher/shmdata-utils.hpp"
#include "./timelaps.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    Timelaps,
    "timelaps",
    "Timelaps",
    "video",
    "reader",
    "Make an image timelaps from raw video stream",
    "LGPL",
    "Nicolas Bouillot");

Timelaps::Timelaps(const std::string &):
    shmcntr_(static_cast<Quiddity *>(this)),
    timelaps_config_{std::string(), std::string()},
    img_path_id_(pmanage<MPtr(&PContainer::make_string)>(
        "imgpath",
        [this](const std::string &val){img_path_ = val; return true;},
        [this](){return img_path_;},
        "Image Path",
        "Path the the jpeg file to write. if empty, the path will be <video_shmdata_path>%05d.jpg",
        img_path_)){
}

bool Timelaps::init() {
  shmcntr_.install_connect_method(
      [this](const std::string &shmpath){return this->on_shmdata_connect(shmpath);},
      [this](const std::string &){return this->on_shmdata_disconnect();},
      [this](){return this->on_shmdata_disconnect();},
      [this](const std::string &caps){return this->can_sink_caps(caps);},
      1);
  return true;
}

bool Timelaps::on_shmdata_disconnect() {
  if (!timelaps_)
    return true;
  timelaps_.reset();
  return true;
}

bool Timelaps::on_shmdata_connect(const std::string &shmpath) {
  timelaps_config_ =
      GstVideoTimelapsConfig(shmpath,
                             img_path_.empty() ? shmpath + "%05d.jpg" : img_path_);
  timelaps_ = std2::make_unique<GstVideoTimelaps>(this, timelaps_config_);
  return true;  // FIXME make videotimelaps testable
}

bool Timelaps::can_sink_caps(const std::string &caps) {
  // assuming timelaps_ is internally using videoconvert as first caps negotiating gst element: 
  return GstUtils::can_sink_caps("videoconvert", caps);
}

}  // namespace switcher
