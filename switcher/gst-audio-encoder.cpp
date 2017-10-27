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
#include "switcher/shmdata-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GstAudioEncoder,
                                     "audioenc",
                                     "Audio Encoder",
                                     "audio",
                                     "writer/reader",
                                     "Encode raw audio stream",
                                     "LGPL",
                                     "Nicolas Bouillot");

GstAudioEncoder::GstAudioEncoder(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      shmcntr_(static_cast<Quiddity*>(this)),
      codecs_(std::make_unique<GstAudioCodec>(static_cast<Quiddity*>(this))) {
  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->on_shmdata_connect(shmpath); },
      [this](const std::string&) { return this->on_shmdata_disconnect(); },
      [this]() { return this->on_shmdata_disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      1);
}

bool GstAudioEncoder::on_shmdata_disconnect() { return codecs_->stop(); }

bool GstAudioEncoder::on_shmdata_connect(const std::string& shmpath) {
  return codecs_->start(shmpath, make_file_name("audio-encoded"));
}

bool GstAudioEncoder::can_sink_caps(const std::string& caps) {
  // assuming codecs_ is internally using audioconvert as first caps negotiating
  // gst element
  return GstUtils::can_sink_caps("audioconvert", caps);
}

}  // namespace switcher
