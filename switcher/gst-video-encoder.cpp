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

#include "./gst-video-encoder.hpp"
#include "switcher/shmdata-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GstVideoEncoder,
                                     "videnc",
                                     "Video Encoder",
                                     "video",
                                     "writer/reader",
                                     "Encode raw video stream",
                                     "LGPL",
                                     "Nicolas Bouillot");

GstVideoEncoder::GstVideoEncoder(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)), shmcntr_(static_cast<Quiddity*>(this)) {
  codecs_ = std::make_unique<GstVideoCodec>(
      static_cast<Quiddity*>(this), std::string(), make_file_name("video-encoded"));
  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->on_shmdata_connect(shmpath); },
      [this](const std::string&) { return this->on_shmdata_disconnect(); },
      [this]() { return this->on_shmdata_disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      1);
}

bool GstVideoEncoder::on_shmdata_disconnect() { return codecs_->stop(); }

bool GstVideoEncoder::on_shmdata_connect(const std::string& shmpath) {
  codecs_->set_shm(shmpath);
  return codecs_->start();
}

bool GstVideoEncoder::can_sink_caps(const std::string& caps) {
  // assuming codecs_ is internally using videoconvert as first caps negotiating
  // gst element:
  return GstUtils::can_sink_caps("videoconvert", caps);
}

}  // namespace switcher
