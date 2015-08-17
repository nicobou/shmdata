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
#include "./gst-video-converter.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    GstVideoConverter,
    "videoconvert",
    "Video converter",
    "video",
    "writer/reader",
    "Convert pixel format of raw video stream",
    "LGPL",
    "Nicolas Bouillot");

GstVideoConverter::GstVideoConverter(const std::string &):
    shmcntr_(static_cast<Quiddity *>(this)),
    custom_props_(std::make_shared<CustomPropertyHelper>()){
}

bool GstVideoConverter::init() {
  converter_ = std2::make_unique<GstPixelFormatConverter>(
      static_cast<Quiddity *>(this),
      custom_props_.get(),
      "Pixel format",
      "Convert to selected pixel format");
  shmcntr_.install_connect_method(
      [this](const std::string &shmpath){return this->on_shmdata_connect(shmpath);},
      [this](const std::string &){return this->on_shmdata_disconnect();},
      [this](){return this->on_shmdata_disconnect();},
      [this](const std::string &caps){return this->can_sink_caps(caps);},
      1);
  return true;
}

bool GstVideoConverter::on_shmdata_disconnect() {
  return converter_->stop();
}

bool GstVideoConverter::on_shmdata_connect(const std::string &shmpath) {
  return converter_->start(shmpath, make_file_name("video-converted"));
}

bool GstVideoConverter::can_sink_caps(const std::string &caps) {
  // assuming codecs_ is internally using videoconvert as first caps negotiating gst element: 
  return GstUtils::can_sink_caps("videoconvert", caps);
}

}  // namespace switcher
