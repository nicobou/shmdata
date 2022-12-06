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
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PortMidiSink,
                                     "midisink",
                                     "Midi (Port Midi)",
                                     "Shmdata to midi",
                                     "LGPL",
                                     "Nicolas Bouillot");
const std::string PortMidiSink::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "midi",
      "description": "Midi stream",
      "can_do": ["audio/midi"]
    }
  ]
}
)");

PortMidiSink::PortMidiSink(quiddity::Config&& conf)
    : Quiddity(
          std::forward<quiddity::Config>(conf),
          {kConnectionSpec,
           [this](const std::string& shmpath, claw::sfid_t) { return on_shmdata_connect(shmpath); },
           [this](claw::sfid_t) { return on_shmdata_disconnect(); }}),
      Startable(this),
      devices_id_(pmanage<MPtr(&property::PBag::make_selection<>)>(
          "device",
          [this](const quiddity::property::IndexOrName& val) {
            output_devices_enum_.select(val);
            int device = stoi(output_devices_enum_.get_attached());
            if (device_ != device) {
              device_ = device;
              if (started_) {
                stop();
                start();
              }
            }
            return true;
          },
          [this]() { return output_devices_enum_.get(); },
          "Output device",
          "MIDI output device to use",
          output_devices_enum_)),
      autostart_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "autostart",
          [this](bool val) {
            autostart_ = val;
            return true;
          },
          [this]() { return autostart_; },
          "Autostart",
          "Start processing on shmdata connect or not",
          autostart_)) {
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
  if (started_) {
    LOGGER_WARN(this->logger, "midisink already started");
    return true;
  }

  BoolLog res = open_output_device(device_);
  if (!res.operator bool()) {
    LOGGER_ERROR(this->logger, res.msg());
    return false;
  }
  started_ = true;

  return true;
}

bool PortMidiSink::stop() {
  if (started_) {
    close_output_device(device_);
    started_ = false;
  }
  return true;
}

bool PortMidiSink::on_shmdata_connect(std::string path) {
  shm_ = std::make_unique<shmdata::Follower>(
      this,
      path,
      [this](void* data, size_t size) { this->on_shmreader_data(data, size); },
      nullptr,
      nullptr,
      shmdata::Stat::kDefaultUpdateInterval,
      shmdata::Follower::Direction::reader,
      true);
  if (autostart_) {
    return pmanage<MPtr(&property::PBag::set_str_str)>("started", "true");
  }
  return true;
}

bool PortMidiSink::on_shmdata_disconnect() {
  shm_.reset();
  if (autostart_) {
    return pmanage<MPtr(&property::PBag::set_str_str)>("started", "false");
  }
  return true;
}


}  // namespace quiddities
}  // namespace switcher
