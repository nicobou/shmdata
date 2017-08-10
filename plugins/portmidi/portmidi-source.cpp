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

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PortMidiSource,
                                     "midisrc",
                                     "Midi (PortMidi)",
                                     "midi",
                                     "writer/hid/device",
                                     "midi to shmdata and properties",
                                     "LGPL",
                                     "Nicolas Bouillot");

PortMidiSource::PortMidiSource(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)) {
  if (input_devices_enum_.empty()) {
    message("ERROR:No MIDI capture device detected.");
    is_valid_ = false;
    return;
  }
  init_startable(this);
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

  install_method("Next MIDI Event To Property",                                 // long name
                 "next_midi_event_to_property",                                 // name
                 "Wait for a MIDI event and make a property for this channel",  // description
                 "success or fail",                                  // return description
                 Method::make_arg_description("Property Long Name",  // first arg long name
                                              "property_long_name",  // fisrt arg name
                                              "string",              // first arg description
                                              nullptr),
                 (Method::method_ptr)&next_midi_event_to_property_method,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, nullptr),
                 this);
  disable_method("next_midi_event_to_property");
  install_method("Remove Midi Property",                             // long name
                 "remove_midi_property",                             // name
                 "remove a property made with Make Property",        // description
                 "success or fail",                                  // return description
                 Method::make_arg_description("Property Long Name",  // first arg long name
                                              "property_long_name",  // fisrt arg name
                                              "string",              // first arg description
                                              nullptr),
                 (Method::method_ptr)&remove_property_method,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, nullptr),
                 this);
  disable_method("remove_midi_property");
  install_method("Map midi channel to property",
                 "map_midi_to_property",
                 "creates a property from a midi channel",
                 "success or fail",
                 Method::make_arg_description("Property Long Name",  // first arg long name
                                              "property_long_name",  // fisrt arg name
                                              "string",              // first arg description
                                              "Midi Status",
                                              "midi_status",
                                              "int",
                                              "Midi data1",
                                              "midi_data1",
                                              "int",
                                              nullptr),
                 (Method::method_ptr)&make_property_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, G_TYPE_INT, G_TYPE_INT, nullptr),
                 this);
  disable_method("map_midi_to_property");
}

bool PortMidiSource::start() {
  shm_ =
      std::make_unique<ShmdataWriter>(this, make_file_name("midi"), sizeof(PmEvent), "audio/midi");
  if (!shm_.get()) {
    message("ERROR:Midi failed to start");
    shm_.reset(nullptr);
    return false;
  }
  pmanage<MPtr(&PContainer::disable)>(devices_id_, disabledWhenStartedMsg);
  open_input_device(std::stoi(input_devices_enum_.get_attached()), on_pm_event, this);
  pmanage<MPtr(&PContainer::enable)>(last_midi_value_id_);
  enable_method("next_midi_event_to_property");
  enable_method("remove_midi_property");
  enable_method("map_midi_to_property");
  return true;
}

bool PortMidiSource::stop() {
  close_input_device(std::stoi(input_devices_enum_.get_attached()));
  pmanage<MPtr(&PContainer::disable)>(last_midi_value_id_, disabledWhenStopedMsg);
  disable_method("next_midi_event_to_property");
  disable_method("remove_midi_property");
  disable_method("map_midi_to_property");
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
    context->qcontainer_->get_switcher()->invoke(context->get_name(),
                                                 "map_midi_to_property",
                                                 nullptr,
                                                 {context->next_property_name_,
                                                  std::to_string(context->last_status_),
                                                  std::to_string(context->last_data1_)});
    context->make_property_for_next_midi_event_ = FALSE;
  }
}

gboolean PortMidiSource::next_midi_event_to_property_method(gchar* long_name, void* user_data) {
  PortMidiSource* context = static_cast<PortMidiSource*>(user_data);
  context->make_property_for_next_midi_event_ = TRUE;
  context->next_property_name_ = long_name;
  return TRUE;
}

gboolean PortMidiSource::remove_property_method(gchar* long_name, void* user_data) {
  PortMidiSource* context = static_cast<PortMidiSource*>(user_data);

  if (context->midi_property_contexts_.find(long_name) == context->midi_property_contexts_.end()) {
    context->debug("property % not found for removing", std::string(long_name));
    return FALSE;
  }

  std::pair<guint, guint> midi_channel;
  for (auto& it : context->midi_channels_) {
    if (g_strcmp0(it.second.c_str(), long_name) == 0) {
      midi_channel = it.first;
      break;
    }
  }

  gchar* prop_name = g_strdup_printf("%u-%u", midi_channel.first, midi_channel.second);
  context->pmanage<MPtr(&PContainer::remove)>(context->prop_ids_[long_name]);
  context->unused_props_specs_[prop_name] = context->prop_ids_[long_name];
  context->prop_ids_.erase(long_name);
  context->midi_channels_.erase(midi_channel);
  context->midi_values_.erase(long_name);
  g_free(prop_name);
  return TRUE;
}

gboolean PortMidiSource::make_property_wrapped(const gchar* property_long_name,
                                               gint last_status,
                                               gint last_data1,
                                               void* user_data) {
  PortMidiSource* context = static_cast<PortMidiSource*>(user_data);
  if (context->make_property(property_long_name, last_status, last_data1)) return TRUE;
  return FALSE;
}

bool PortMidiSource::make_property(std::string property_long_name,
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
