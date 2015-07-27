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
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    PortMidiSink,
    "midisink",
    "Midi (Port Midi)",
    "midi",
    "reader/device",
    "shmdata to midi",
    "LGPL",
    "Nicolas Bouillot");

PortMidiSink::PortMidiSink(const std::string &):
    shmcntr_(static_cast<Quiddity *>(this)),
    custom_props_(new CustomPropertyHelper()) {
}

bool PortMidiSink::init() {
  init_startable(this);
  shmcntr_.install_connect_method(
      [this](const std::string &shmpath){return this->connect(shmpath);},
      [this](const std::string &){return this->disconnect();},
      [this](){return this->disconnect();},
      [this](const std::string &caps){return this->can_sink_caps(caps);},
      1);
  device_ = output_devices_enum_[0].value;
  devices_enum_spec_ =
      custom_props_->make_enum_property("device",
                                        "Enumeration of MIDI capture devices",
                                        device_,
                                        output_devices_enum_,
                                        (GParamFlags) G_PARAM_READWRITE,
                                        PortMidiSink::set_device,
                                        PortMidiSink::get_device, this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            devices_enum_spec_, "device", "Capture Device");
  return true;
}

void PortMidiSink::on_shmreader_data(void *data, size_t /*size */) {
  PmEvent *event = static_cast<PmEvent *>(data);
  push_midi_message(device_,
                    Pm_MessageStatus(event->message),
                    Pm_MessageData1(event->message),
                    Pm_MessageData2(event->message));
}

void PortMidiSink::set_device(const gint value, void *user_data) {
  PortMidiSink *context = static_cast<PortMidiSink *>(user_data);
  context->device_ = value;
}

gint PortMidiSink::get_device(void *user_data) {
  PortMidiSink *context = static_cast<PortMidiSink *>(user_data);
  return context->device_;
}

bool PortMidiSink::start() {
  disable_property("device");
  open_output_device(device_);
  // FIXME the following might not be necessary
  gint stat = 165;
  gint data1 = 1;
  gint data2 = 67;
  push_midi_message(device_,
                    (unsigned char)stat,
                    (unsigned char)data1,
                    (unsigned char)data2);
  return true;
}

bool PortMidiSink::stop() {
  close_output_device(device_);
  enable_property("device");
  return true;
}

bool PortMidiSink::connect(std::string path) {
  shm_.reset(new ShmdataFollower(this,
                                 path,
                                 [this](void *data, size_t size){
                                   this->on_shmreader_data(data, size);
                                 }));
  return true;
}

bool PortMidiSink::disconnect() {
  shm_.reset(nullptr);
  return true;
}

bool PortMidiSink::can_sink_caps(std::string caps) {
  return (0 == caps.find("audio/midi"));
}

}  // namespace switcher
