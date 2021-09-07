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

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GstVideoEncoder,
                                     "videnc",
                                     "Video Encoder",
                                     "video",
                                     "writer/reader",
                                     "Encode raw video stream",
                                     "LGPL",
                                     "Nicolas Bouillot");

const std::string GstVideoEncoder::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "video",
      "description": "Video stream to encode",
      "can_do": ["video/x-raw"]
    }
  ],
"writer":
  [
    {
      "label": "video-encoded",
      "description": "Encoded video stream",
      "can_do": ["any"]
    }
  ]
}
)");

GstVideoEncoder::GstVideoEncoder(quiddity::Config&& conf)
    : Quiddity(
          std::forward<quiddity::Config>(conf),
          {kConnectionSpec,
           [this](const std::string& shmpath, claw::sfid_t) { return on_shmdata_connect(shmpath); },
           [this](claw::sfid_t) { return on_shmdata_disconnect(); }}),
      codecs_(std::make_unique<gst::VideoCodec>(
          static_cast<Quiddity*>(this),
          std::string(),
          claw_.get_shmpath_from_writer_label("video-encoded"))) {}

bool GstVideoEncoder::on_shmdata_disconnect() { return codecs_->stop(); }

bool GstVideoEncoder::on_shmdata_connect(const std::string& shmpath) {
  codecs_->set_shm(shmpath);
  return codecs_->start();
}

}  // namespace quiddities
}  // namespace switcher
