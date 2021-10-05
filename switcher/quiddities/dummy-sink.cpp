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
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(DummySink,
                                     "dummysink",
                                     "Dummy Sink Quiddity",
                                     "Quiddity for testing the reading of shmdata",
                                     "LGPL",
                                     "Nicolas Bouillot");

const std::string DummySink::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "default",
      "description": "Read all shmdata for monitoring purpose",
      "can_do": [ "all" ]
    }
  ]
}
)");

DummySink::DummySink(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf),
               {kConnectionSpec,
                [this](const std::string& shmpath, claw::sfid_t) { return connect(shmpath); },
                [this](claw::sfid_t) { return disconnect(); }}),
      frame_received_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "frame-received",
          nullptr,
          [this]() { return frame_received_; },
          "Frame Received",
          "A Frame has been received on the shmdata",
          frame_received_)) {}

bool DummySink::connect(const std::string& path) {
  shm_.reset();
  shm_ = std::make_unique<shmdata::Follower>(this, path, [this](void*, size_t) {
    if (!frame_received_) {
      {
        auto lock = pmanage<MPtr(&property::PBag::get_lock)>(frame_received_id_);
        frame_received_ = true;
      }
      pmanage<MPtr(&property::PBag::notify)>(frame_received_id_);
    }
  });
  return true;
}

bool DummySink::disconnect() {
  shm_.reset();
  return true;
}

}  // namespace quiddities
}  // namespace switcher
