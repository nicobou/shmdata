/*
 * This file is part of switcher-portmidi.
 *
 * switcher-portmidi is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_PORTMIDI_SINK_H__
#define __SWITCHER_PORTMIDI_SINK_H__

#include <memory>
#include "./portmidi-devices.hpp"
#include "switcher/quiddity/quiddity.hpp"
#include "switcher/quiddity/startable.hpp"
#include "switcher/shmdata/shmdata-connector.hpp"
#include "switcher/shmdata/shmdata-follower.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;
class PortMidiSink : public Quiddity, public Startable, public PortMidi {
 public:
  PortMidiSink(quiddity::Config&&);
  ~PortMidiSink() = default;
  PortMidiSink(const PortMidiSink&) = delete;
  PortMidiSink& operator=(const PortMidiSink&) = delete;

 private:
  bool start() final;
  bool stop() final;
  // segment callback
  bool on_shmdata_connect(std::string path);
  bool on_shmdata_disconnect();
  bool can_sink_caps(std::string caps);
  // shmdata any callback
  void on_shmreader_data(void* data, size_t data_size);

  bool started_{false};
  // registering connect/disconnect/can_sink_caps:
  ShmdataConnector shmcntr_;
  // shmdata follower
  std::unique_ptr<ShmdataFollower> shm_{nullptr};

  int device_{0};
  property::prop_id_t devices_id_{0};
  bool autostart_{false};
  property::prop_id_t autostart_id_;
};

SWITCHER_DECLARE_PLUGIN(PortMidiSink);
}  // namespace quiddities
}  // namespace switcher
#endif
