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

#include <time.h>
#include <switcher/std2.hpp>
#include <switcher/quiddity-manager.hpp>
#include <switcher/quiddity-manager-impl.hpp>
#include <switcher/scope-exit.hpp>
#include "./portmidi-source.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    PortMidiSource,
    "midisrc",
    "Midi (PortMidi)",
    "midi",
    "writer/hid/device",
    "midi to shmdata and properties",
    "LGPL",
    "Nicolas Bouillot");

PortMidiSource::PortMidiSource(const std::string &):
    last_status_(-1),
    last_data1_(-1),
    last_data2_(-1),
    custom_props_(new CustomPropertyHelper()),
    devices_enum_spec_(nullptr),
    device_(0),
    midi_value_spec_(nullptr),
    make_property_for_next_midi_event_(FALSE),
    next_property_name_(),
    prop_specs_(),
    midi_property_contexts_(),
    midi_channels_(),
    midi_values_(),
    unused_props_specs_() {
}

bool PortMidiSource::init() {
  if (input_devices_enum_[0].value_name == nullptr) {
    g_debug("no MIDI capture device detected");
    return false;
  }
  init_startable(this);
  device_ = input_devices_enum_[0].value;
  devices_enum_spec_ =
      custom_props_->make_enum_property("device",
                                        "Enumeration of MIDI capture devices",
                                        device_,
                                        input_devices_enum_,
                                        (GParamFlags) G_PARAM_READWRITE,
                                        PortMidiSource::set_device,
                                        PortMidiSource::get_device, this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            devices_enum_spec_, "device", "Capture Device");
  midi_value_spec_ =
      custom_props_->make_int_property("last-midi-value",
                                       "The last midi value from the device",
                                       0,
                                       127,
                                       0,
                                       (GParamFlags) G_PARAM_READABLE,
                                       nullptr,
                                       get_midi_value,
                                       this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            midi_value_spec_,
                            "last-midi-value",
                            "Last Midi Value");
  disable_property("last-midi-value");
  install_method("Next MIDI Event To Property",  // long name
                 "next_midi_event_to_property",  // name
                 "Wait for a MIDI event and make a property for this channel",  // description
                 "success or fail",  // return description
                 Method::make_arg_description("Property Long Name",  // first arg long name
                                              "property_long_name",  // fisrt arg name
                                              "string",  // first arg description
                                              nullptr),
                 (Method::method_ptr) &next_midi_event_to_property_method,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, nullptr),
                 this);
  disable_method("next_midi_event_to_property");
  install_method("Remove Midi Property",      // long name
                 "remove_midi_property",      // name
                 "remove a property made with Make Property",  // description
                 "success or fail",   // return description
                 Method::make_arg_description("Property Long Name",   // first arg long name
                                              "property_long_name",   // fisrt arg name
                                              "string",       // first arg description
                                              nullptr),
                 (Method::method_ptr) &remove_property_method,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, nullptr),
                 this);
  disable_method("remove_midi_property");
  install_method("Map midi channel to property",
                 "map_midi_to_property",
                 "creates a property from a midi channel",
                 "success or fail",
                 Method::make_arg_description("Property Long Name",   // first arg long name
                                              "property_long_name",   // fisrt arg name
                                              "string",       // first arg description
                                              "Midi Status",
                                              "midi_status",
                                              "int",
                                              "Midi data1",
                                              "midi_data1",
                                              "int",
                                              nullptr),
                 (Method::method_ptr) &make_property_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   G_TYPE_INT,
                                                   G_TYPE_INT,
                                                   nullptr),
                 this);
  disable_method("map_midi_to_property");
  shm_ = std2::make_unique<ShmdataWriter>(this,
                                          make_file_name("midi"),
                                          sizeof(PmEvent),
                                          "audio/midi");
  if(!shm_.get()) {
    g_warning("midi failed to start");
    shm_.reset(nullptr);
    return false;
  }
  return true;
}

bool PortMidiSource::start() {
  disable_property("device");
  open_input_device(device_, on_pm_event, this);
  enable_property("last-midi-value");
  enable_method("next_midi_event_to_property");
  enable_method("remove_midi_property");
  enable_method("map_midi_to_property");
  return true;
}

bool PortMidiSource::stop() {
  close_input_device(device_);
  disable_property("last-midi-value");
  disable_method("next_midi_event_to_property");
  disable_method("remove_midi_property");
  disable_method("map_midi_to_property");
  enable_property("device");
  return true;
}

void PortMidiSource::set_device(const gint value, void *user_data) {
  PortMidiSource *context = static_cast<PortMidiSource *>(user_data);
  context->device_ = value;
}

gint PortMidiSource::get_device(void *user_data) {
  PortMidiSource *context = static_cast<PortMidiSource *>(user_data);
  return context->device_;
}

void PortMidiSource::on_pm_event(PmEvent *event, void *user_data) {
  PortMidiSource *context = static_cast<PortMidiSource *>(user_data);

  PmEvent *tmp_event = (PmEvent *) g_malloc(sizeof(PmEvent));
  tmp_event->message = event->message;
  tmp_event->timestamp = event->timestamp;
  context->shm_->writer(&shmdata::Writer::copy_to_shm, tmp_event, sizeof(PmEvent));
  context->shm_->bytes_written(sizeof(PmEvent));
  g_free(tmp_event);

  guint status = Pm_MessageStatus(event->message);
  guint data1 = Pm_MessageData1(event->message);
  guint data2 = Pm_MessageData2(event->message);

  context->last_status_ = (gint) status;
  context->last_data1_ = (gint) data1;
  context->last_data2_ = (gint) data2;
  context->custom_props_->
      notify_property_changed(context->midi_value_spec_);

  // g_print ("to shm:  %u %u %u event ts %d tmp_event_ts %d\n",
  //        status,
  //        data1,
  //        data2,
  //      event->timestamp);

  // updating property if needed
  if (context->midi_channels_.find(std::make_pair(status, data1)) !=
      context->midi_channels_.end()) {
    std::string prop_long_name =
        context->midi_channels_[std::make_pair(status, data1)];
    context->midi_values_[prop_long_name] = data2;
    context->custom_props_->
        notify_property_changed(context->prop_specs_[prop_long_name]);
  }

  // making property if needed
  if (context->make_property_for_next_midi_event_){
    QuiddityManager_Impl::ptr manager = context->manager_impl_.lock();
    if (manager){
      manager->get_root_manager()->invoke(
          context->get_name(),
          "map_midi_to_property",
          nullptr,
          {context->next_property_name_,
                std::to_string(context->last_status_),
                std::to_string(context->last_data1_)});
    } else {
      g_warning("no manager in portmidi-source");
    }
    context->make_property_for_next_midi_event_ = FALSE;
  }
}

gboolean
PortMidiSource::next_midi_event_to_property_method(gchar *long_name,
                                                   void *user_data) {
  PortMidiSource *context = static_cast<PortMidiSource *>(user_data);
  context->make_property_for_next_midi_event_ = TRUE;
  context->next_property_name_ = long_name;
  return TRUE;
}

gint PortMidiSource::get_midi_property_value(void *user_data) {
  MidiPropertyContext *context = static_cast<MidiPropertyContext *>(user_data);
  return context->port_midi_source_->
      midi_values_[context->property_long_name_];
}

gboolean
PortMidiSource::remove_property_method(gchar *long_name,
                                       void *user_data) {
  PortMidiSource *context = static_cast<PortMidiSource *>(user_data);

  if (context->midi_property_contexts_.find(long_name) ==
      context->midi_property_contexts_.end()) {
    g_debug("property %s not found for removing", long_name);
    return FALSE;
  }

  std::pair<guint, guint> midi_channel;
  for (auto &it : context->midi_channels_) {
    if (g_strcmp0(it.second.c_str(), long_name) == 0) {
      midi_channel = it.first;
      break;
    }
  }

  gchar *prop_name =
      g_strdup_printf("%u-%u", midi_channel.first, midi_channel.second);
  context->uninstall_property(prop_name);
  context->unused_props_specs_[prop_name] = context->prop_specs_[long_name];
  context->prop_specs_.erase(long_name);
  context->midi_channels_.erase(midi_channel);
  context->midi_values_.erase(long_name);
  g_free(prop_name);
  return TRUE;
}

gboolean PortMidiSource::make_property_wrapped(const gchar *property_long_name,
                                               gint last_status,
                                               gint last_data1,
                                               void *user_data){
  PortMidiSource *context = static_cast<PortMidiSource *>(user_data);
  if (context->make_property(property_long_name, last_status, last_data1))
    return TRUE;
  return FALSE;  
}

bool PortMidiSource::make_property(std::string property_long_name,
                                   gint last_status,
                                   gint last_data1) {
  if (midi_channels_.find(std::make_pair(last_status, last_data1))
      != midi_channels_.end()) {
    g_debug("Midi Channels %u %u is already a property (is currently named %s)",
            last_status, last_data1,
            midi_channels_.find(std::make_pair(last_status, last_data1))->second.c_str());
    return false;
  }
  midi_channels_[std::make_pair(last_status, last_data1)] =
      property_long_name;
  std::string prop_name(std::to_string(last_status) + "-" + std::to_string(last_data1));
  midi_values_[property_long_name] = last_data2_;
  if (unused_props_specs_.find(prop_name) == unused_props_specs_.end()) {
    MidiPropertyContext midi_property_context;
    midi_property_context.port_midi_source_ = this;
    midi_property_context.property_long_name_ = property_long_name;
    midi_property_contexts_[property_long_name] = midi_property_context;
    prop_specs_[property_long_name] =
        custom_props_->make_int_property(prop_name.c_str(),
                                         "midi value",
                                         0,
                                         127,
                                         0,
                                         (GParamFlags) G_PARAM_READABLE,
                                         nullptr,
                                         get_midi_property_value,
                                         &midi_property_contexts_
                                         [property_long_name]);
  }
  else {
    prop_specs_[property_long_name] = unused_props_specs_[prop_name];
    unused_props_specs_.erase(prop_name);
  }
  install_property_by_pspec(custom_props_->get_gobject(),
                            prop_specs_[property_long_name],
                            prop_name, property_long_name.c_str());
  return true;
}

gint PortMidiSource::get_midi_value(void *user_data) {
  PortMidiSource *context = static_cast<PortMidiSource *>(user_data);
  return context->last_data2_;
}
}
