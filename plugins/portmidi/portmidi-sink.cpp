/*
 * This file is part of switcher-portmidi.
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

#include "./portmidi-sink.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PortMidiSink,
                                     "midisink",
                                     "Midi (Port Midi)",
                                     "midi",
                                     "reader/device",
                                     "shmdata to midi",
                                     "LGPL",
                                     "Nicolas Bouillot");

PortMidiSink::PortMidiSink(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)), shmcntr_(static_cast<Quiddity*>(this)) {
  init_startable(this);
  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->connect(shmpath); },
      [this](const std::string&) { return this->disconnect(); },
      [this]() { return this->disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      1);

  devices_id_ = pmanage<MPtr(&PContainer::make_selection<>)>(
      "device",
      [this](const IndexOrName& val) {
        output_devices_enum_.select(val);
        device_ = stoi(output_devices_enum_.get_attached());
        return true;
      },
      [this]() { return output_devices_enum_.get(); },
      "Capture device",
      "MIDI capture device to use",
      output_devices_enum_);
  device_ = stoi(output_devices_enum_.get_attached());
}

void PortMidiSink::on_shmreader_data(void* data, size_t /*size */) {
  PmEvent* event = static_cast<PmEvent*>(data);
  push_midi_message(device_,
                    Pm_MessageStatus(event->message),
                    Pm_MessageData1(event->message),
                    Pm_MessageData2(event->message));
}

bool PortMidiSink::start() {
  pmanage<MPtr(&PContainer::disable)>(devices_id_, disabledWhenStartedMsg);
  open_output_device(device_);
  // FIXME the following might not be necessary
  gint stat = 165;
  gint data1 = 1;
  gint data2 = 67;
  push_midi_message(device_, (unsigned char)stat, (unsigned char)data1, (unsigned char)data2);
  return true;
}

bool PortMidiSink::stop() {
  close_output_device(device_);
  pmanage<MPtr(&PContainer::enable)>(devices_id_);
  return true;
}

bool PortMidiSink::connect(std::string path) {
  shm_ = std::make_unique<ShmdataFollower>(
      this,
      path,
      [this](void* data, size_t size) { this->on_shmreader_data(data, size); },
      nullptr,
      nullptr,
      ShmdataStat::kDefaultUpdateInterval);
  return true;
}

bool PortMidiSink::disconnect() {
  shm_.reset(nullptr);
  return true;
}

bool PortMidiSink::can_sink_caps(std::string caps) { return (0 == caps.find("audio/midi")); }

}  // namespace switcher
