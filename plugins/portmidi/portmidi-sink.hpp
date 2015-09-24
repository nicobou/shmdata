/*
 * This file is part of switcher-myplugin.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
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
#include "switcher/quiddity.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "./portmidi-devices.hpp"

namespace switcher {
class PortMidiSink:public Quiddity, public StartableQuiddity, public PortMidi {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PortMidiSink);
  PortMidiSink(const std::string &);
  ~PortMidiSink() = default;
  PortMidiSink(const PortMidiSink &) = delete;
  PortMidiSink &operator=(const PortMidiSink &) = delete;

 private:
  // registering connect/disconnect/can_sink_caps:
  ShmdataConnector shmcntr_;
  // shmdata follower
  std::unique_ptr<ShmdataFollower> shm_{nullptr};
  PContainer::prop_id_t devices_id_{0};
  int device_{0};
  bool init() final;
  bool start() final;
  bool stop() final;
  // segment callback
  bool connect(std::string path);
  bool disconnect();
  bool can_sink_caps(std::string caps);
  // shmdata any callback
  void on_shmreader_data(void *data, size_t data_size);
};

SWITCHER_DECLARE_PLUGIN(PortMidiSink);
}  // namespace switcher
#endif
