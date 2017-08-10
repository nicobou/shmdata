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

#include "./dummy-sink.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(DummySink,
                                     "dummysink",
                                     "Dummy Sink Quiddity",
                                     "utils",
                                     "",
                                     "Quiddity for testing the reading of shmdatas",
                                     "LGPL",
                                     "Nicolas Bouillot");

DummySink::DummySink(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      frame_received_id_(
          pmanage<MPtr(&PContainer::make_bool)>("frame-received",
                                                nullptr,
                                                [this]() { return frame_received_; },
                                                "Frame Received",
                                                "A Frame has been receivedon the shmdata",
                                                frame_received_)),
      shmcntr_(static_cast<Quiddity*>(this)) {
  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->connect(shmpath); },
      [this](const std::string&) { return this->disconnect(); },
      [this]() { return this->disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      1);
}

bool DummySink::connect(const std::string& path) {
  shm_.reset();
  shm_ = std::make_unique<ShmdataFollower>(this, path, [this](void*, size_t) {
    if (!frame_received_) {
      {
        auto lock = pmanage<MPtr(&PContainer::get_lock)>(frame_received_id_);
        frame_received_ = true;
      }
      pmanage<MPtr(&PContainer::notify)>(frame_received_id_);
    }
  });
  return true;
}

bool DummySink::disconnect() {
  shm_.reset(nullptr);
  return true;
}

bool DummySink::can_sink_caps(const std::string&) { return true; }

}  // namespace switcher
