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

#include "./portmidi-source.hpp"
#include <time.h>
#include <switcher/quiddity-container.hpp>
#include <switcher/scope-exit.hpp>
#include <switcher/switcher.hpp>
#include "switcher/information-tree-json.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PortMidiSource,
                                     "midisrc",
                                     "Midi (PortMidi)",
                                     "midi",
                                     "writer/hid/device",
                                     "Midi to shmdata and properties",
                                     "LGPL",
                                     "Nicolas Bouillot");

PortMidiSource::PortMidiSource(quid::Config&& conf) : Quiddity(std::forward<quid::Config>(conf)) {
  if (input_devices_enum_.empty()) {
    message("ERROR:No MIDI capture device detected.");
    is_valid_ = false;
    return;
  }
  init_startable(this);
  register_writer_suffix("midi");
  devices_id_ =
      pmanage<MPtr(&PContainer::make_selection<>)>("device",
                                                   [this](const IndexOrName& val) {
                                                     input_devices_enum_.select(val);
                                                     return true;
                                                   },
                                                   [this]() { return input_devices_enum_.get(); },
                                                   "Capture device",
                                                   "MIDI capture devices to use",
                                                   input_devices_enum_);
  last_midi_value_id_ =
      pmanage<MPtr(&PContainer::make_int)>("last-midi-value",
                                           nullptr,
                                           [this]() { return last_data2_; },
                                           "Last MIDI value",
                                           "Last MIDI value seen on capture device",
                                           0,
                                           0,
                                           127);
  pmanage<MPtr(&PContainer::disable)>(last_midi_value_id_, disabledWhenStopedMsg);

  auto next_midi_event_to_property_id_ =
      mmanage<MPtr(&MContainer::make_method<std::function<bool(std::string)>>)>(
          "next_midi_event_to_property",
          JSONSerializer::deserialize(
              R"(
                  {
                   "name" : "Next MIDI Event To Property",
                   "description" : "Wait for a MIDI event and make a property for this channel",
                   "arguments" : [
                     {
                        "long name" : "Property Long Name",
                        "description" : "string"
                     }
                   ]
                  }
              )"),
          [this](const std::string& prop_name) {
            return next_midi_event_to_property_method(prop_name);
          });
  mmanage<MPtr(&MContainer::disable)>(next_midi_event_to_property_id_,
                                      StartableQuiddity::disabledWhenStopedMsg);

  auto remove_midi_property_id_ =
      mmanage<MPtr(&MContainer::make_method<std::function<bool(std::string)>>)>(
          "remove_midi_property",
          JSONSerializer::deserialize(
              R"(
                  {
                   "name" : "Remove Midi Property",
                   "description" : "remove a property made with Make Property",
                   "arguments" : [
                     {
                        "long name" : "Property Long Name",
                        "description" : "string"
                     }
                   ]
                  }
              )"),
          [this](const std::string& prop_name) { return remove_property_method(prop_name); });
  mmanage<MPtr(&MContainer::disable)>(remove_midi_property_id_,
                                      StartableQuiddity::disabledWhenStopedMsg);

  using map_midi_to_prop_t = std::function<bool(std::string, int, int)>;
  auto map_midi_to_property_id_ = mmanage<MPtr(&MContainer::make_method<map_midi_to_prop_t>)>(
      "map_midi_to_property",
      JSONSerializer::deserialize(
          R"(
                  {
                   "name" : "Map midi channel to property",
                   "description" :  "creates a property from a midi channel",
                   "arguments" : [
                     {
                        "long name" : "Property Long Name",
                        "description" : "string"
                     },
                     {
                        "long name" : "Midi Status",
                        "description" : "int"
                     },
                     {
                        "long name" : "Midi data1",
                        "description" : "int"
                     }
                   ]
                  }
              )"),
      [this](const std::string& prop_name, int status, int data) {
        return make_property(prop_name, status, data);
      });
  mmanage<MPtr(&MContainer::disable)>(map_midi_to_property_id_,
                                      StartableQuiddity::disabledWhenStopedMsg);
}

bool PortMidiSource::start() {
  shm_ = std::make_unique<ShmdataWriter>(this, make_shmpath("midi"), sizeof(PmEvent), "audio/midi");
  if (!shm_.get()) {
    message("ERROR:Midi failed to start");
    shm_.reset(nullptr);
    return false;
  }
  pmanage<MPtr(&PContainer::disable)>(devices_id_, disabledWhenStartedMsg);
  open_input_device(std::stoi(input_devices_enum_.get_attached()), on_pm_event, this);
  pmanage<MPtr(&PContainer::enable)>(last_midi_value_id_);
  mmanage<MPtr(&MContainer::enable)>(next_midi_event_to_property_id_);
  mmanage<MPtr(&MContainer::enable)>(remove_midi_property_id_);
  mmanage<MPtr(&MContainer::enable)>(map_midi_to_property_id_);
  return true;
}

bool PortMidiSource::stop() {
  close_input_device(std::stoi(input_devices_enum_.get_attached()));
  pmanage<MPtr(&PContainer::disable)>(last_midi_value_id_, disabledWhenStopedMsg);
  mmanage<MPtr(&MContainer::disable)>(next_midi_event_to_property_id_,
                                      StartableQuiddity::disabledWhenStopedMsg);
  mmanage<MPtr(&MContainer::disable)>(remove_midi_property_id_,
                                      StartableQuiddity::disabledWhenStopedMsg);
  mmanage<MPtr(&MContainer::disable)>(map_midi_to_property_id_,
                                      StartableQuiddity::disabledWhenStopedMsg);
  pmanage<MPtr(&PContainer::enable)>(devices_id_);
  shm_.reset(nullptr);
  return true;
}

void PortMidiSource::on_pm_event(PmEvent* event, void* user_data) {
  PortMidiSource* context = static_cast<PortMidiSource*>(user_data);

  PmEvent* tmp_event = (PmEvent*)g_malloc(sizeof(PmEvent));
  tmp_event->message = event->message;
  tmp_event->timestamp = event->timestamp;
  context->shm_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(tmp_event, sizeof(PmEvent));
  context->shm_->bytes_written(sizeof(PmEvent));
  g_free(tmp_event);

  guint status = Pm_MessageStatus(event->message);
  guint data1 = Pm_MessageData1(event->message);
  guint data2 = Pm_MessageData2(event->message);

  context->last_status_ = (gint)status;
  context->last_data1_ = (gint)data1;

  {
    auto lock = context->pmanage<MPtr(&PContainer::get_lock)>(context->last_midi_value_id_);
    context->last_data2_ = (gint)data2;
  }
  context->pmanage<MPtr(&PContainer::notify)>(context->last_midi_value_id_);
  // updating property if needed
  if (context->midi_channels_.find(std::make_pair(status, data1)) !=
      context->midi_channels_.end()) {
    std::string prop_long_name = context->midi_channels_[std::make_pair(status, data1)];
    context->midi_values_[prop_long_name] = data2;
    context->pmanage<MPtr(&PContainer::notify)>(context->prop_ids_[prop_long_name]);
  }

  // making property if needed
  if (context->make_property_for_next_midi_event_) {
    context->make_property(
        context->next_property_name_, context->last_status_, context->last_data1_);
    context->make_property_for_next_midi_event_ = FALSE;
  }
}

bool PortMidiSource::next_midi_event_to_property_method(const std::string& long_name) {
  make_property_for_next_midi_event_ = TRUE;
  next_property_name_ = long_name;
  return true;
}

bool PortMidiSource::remove_property_method(const std::string& long_name) {
  if (midi_property_contexts_.find(long_name) == midi_property_contexts_.end()) {
    debug("property % not found for removing", std::string(long_name));
    return false;
  }

  std::pair<guint, guint> midi_channel;
  for (auto& it : midi_channels_) {
    if (it.second == long_name) {
      midi_channel = it.first;
      break;
    }
  }

  gchar* prop_name = g_strdup_printf("%u-%u", midi_channel.first, midi_channel.second);
  pmanage<MPtr(&PContainer::remove)>(prop_ids_[long_name]);
  unused_props_specs_[prop_name] = prop_ids_[long_name];
  prop_ids_.erase(long_name);
  midi_channels_.erase(midi_channel);
  midi_values_.erase(long_name);
  g_free(prop_name);
  return true;
}

bool PortMidiSource::make_property(const std::string& property_long_name,
                                   gint last_status,
                                   gint last_data1) {
  if (midi_channels_.find(std::make_pair(last_status, last_data1)) != midi_channels_.end()) {
    message(
        "ERROR:Midi Channels % % is already a property (is currently named "
        "%)",
        std::to_string(last_status),
        std::to_string(last_data1),
        midi_channels_.find(std::make_pair(last_status, last_data1))->second);
    return false;
  }
  midi_channels_[std::make_pair(last_status, last_data1)] = property_long_name;
  std::string prop_name(std::to_string(last_status) + "-" + std::to_string(last_data1));
  midi_values_[property_long_name] = last_data2_;
  if (unused_props_specs_.find(prop_name) == unused_props_specs_.end()) {
    MidiPropertyContext midi_property_context;
    midi_property_context.port_midi_source_ = this;
    midi_property_context.property_long_name_ = property_long_name;
    midi_property_contexts_[property_long_name] = midi_property_context;
    prop_ids_[property_long_name] = pmanage<MPtr(&PContainer::make_int)>(
        prop_name,
        nullptr,
        [this, property_long_name]() { return midi_values_[property_long_name]; },
        property_long_name,
        property_long_name,
        0,
        0,
        127);
  } else {
    prop_ids_[property_long_name] = unused_props_specs_[prop_name];
    unused_props_specs_.erase(prop_name);
  }
  return true;
}

}  // namespace switcher
