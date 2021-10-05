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

#include "./gst-audio-encoder.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GstAudioEncoder,
                                     "audioenc",
                                     "Audio Encoder",
                                     "Encode raw audio stream",
                                     "LGPL",
                                     "Nicolas Bouillot");

const std::string GstAudioEncoder::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "rawaudio",
      "description": "The audio stream to encode",
      "can_do": ["audio/x-raw"]
    }
  ],
"writer":
  [
    {
      "label": "audio-encoded",
      "description": "The encoded audio",
      "can_do": ["any"]
    }
  ]
}
)");

GstAudioEncoder::GstAudioEncoder(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf),
               {kConnectionSpec,
                [this](const std::string& shmpath, claw::sfid_t sfid) {
                  return on_shmdata_connect(shmpath);
                },
                [this](claw::sfid_t sfid) { return on_shmdata_disconnect(); }}),
      codecs_(std::make_unique<gst::AudioCodec>(static_cast<Quiddity*>(this))) {}

bool GstAudioEncoder::on_shmdata_disconnect() { return codecs_->stop(); }

bool GstAudioEncoder::on_shmdata_connect(const std::string& shmpath) {
  return codecs_->start(shmpath, claw_.get_shmpath_from_writer_label("audio-encoded"));
}

}  // namespace quiddities
}  // namespace switcher
