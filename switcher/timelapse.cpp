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
#include "./timelapse.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    Timelapse,
    "timelapse",
    "Timelapse",
    "video",
    "reader",
    "Make an image timelapse from raw video stream",
    "LGPL",
    "Nicolas Bouillot");

Timelapse::Timelapse(const std::string &):
    shmcntr_(static_cast<Quiddity *>(this)),
    timelapse_config_{std::string(), std::string()},
    img_path_id_(pmanage<MPtr(&PContainer::make_string)>(
        "imgpath",
        [this](const std::string &val){img_path_ = val; return true;},
        [this](){return img_path_;},
        "Image Path",
        "Path the the jpeg file to write. if empty, the path will be <video_shmdata_path>%05d.jpg",
        img_path_)),
    framerate_id_(pmanage<MPtr(&PContainer::make_fraction)>(
        "framerate",
        [this](const Fraction &val){
          framerate_ = val; return true;
      },
        [this](){return framerate_;},
        "Framerate",
        "Number of image to be produced by seconds",
        framerate_,
        1, 1,  // min num/denom
        60, 5)  // max num/denom
                 ){
    }

bool Timelapse::init() {
  shmcntr_.install_connect_method(
      [this](const std::string &shmpath){return this->on_shmdata_connect(shmpath);},
      [this](const std::string &){return this->on_shmdata_disconnect();},
      [this](){return this->on_shmdata_disconnect();},
      [this](const std::string &caps){return this->can_sink_caps(caps);},
      1);
  return true;
}

bool Timelapse::on_shmdata_disconnect() {
  if (!timelapse_)
    return true;
  timelapse_.reset();
  return true;
}

bool Timelapse::on_shmdata_connect(const std::string &shmpath) {
  timelapse_config_ =
      GstVideoTimelapseConfig(shmpath,
                              img_path_.empty() ? shmpath + "%05d.jpg" : img_path_);
  timelapse_config_.framerate_num_ = framerate_.numerator();
  timelapse_config_.framerate_denom_ = framerate_.denominator();
  timelapse_ = std2::make_unique<GstVideoTimelapse>(this, timelapse_config_);
  return true;  // FIXME make videotimelapse testable
}

bool Timelapse::can_sink_caps(const std::string &caps) {
  // assuming timelapse_ is internally using videoconvert as first caps negotiating gst element: 
  return GstUtils::can_sink_caps("videoconvert", caps);
}

}  // namespace switcher
