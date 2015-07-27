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

#ifndef __SWITCHER_PORTMIDI_SOURCE_H__
#define __SWITCHER_PORTMIDI_SOURCE_H__

#include "switcher/quiddity.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/custom-property-helper.hpp"
#include "switcher/shmdata-writer.hpp"
#include "./portmidi-devices.hpp"

namespace switcher {
class PortMidiSource: public Quiddity,
                      public StartableQuiddity,
                      public PortMidi {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PortMidiSource);
  PortMidiSource(const std::string &);
  ~PortMidiSource() = default;
  PortMidiSource(const PortMidiSource &) = delete;
  PortMidiSource &operator=(const PortMidiSource &) = delete;

 private:
  typedef struct {
    PortMidiSource *port_midi_source_{nullptr};
    std::string property_long_name_{};
  } MidiPropertyContext;

  std::unique_ptr<ShmdataWriter> shm_{nullptr};
  gint last_status_;
  gint last_data1_;
  gint last_data2_;
  // properties
  CustomPropertyHelper::ptr custom_props_;
  // device selection
  GParamSpec *devices_enum_spec_;
  gint device_;
  // last midi value property
  GParamSpec *midi_value_spec_;
  gboolean make_property_for_next_midi_event_;
  std::string next_property_name_;
  std::map<std::string, GParamSpec *>prop_specs_;
  // this is persistent to the quiddity:
  std::map<std::string, MidiPropertyContext> midi_property_contexts_;  
  std::map<std::pair<guint, guint>, std::string> midi_channels_;
  std::map<std::string, guint> midi_values_;
  // using property name instead of long name:
  std::map<std::string, GParamSpec *>unused_props_specs_;

  bool init() final;
  bool start() final;
  bool stop() final;

  bool make_property(std::string property_long_name, gint last_status, gint last_data);
  static void set_device(const gint value, void *user_data);
  static gint get_device(void *user_data);
  static gint get_midi_value(void *user_data);
  // midi properties
  static gboolean next_midi_event_to_property_method(gchar *long_name,
                                                     void *user_data);
  static gboolean remove_property_method(gchar *long_name,
                                         void *user_data);
  static gint get_midi_property_value(void *user_data);
  static void on_pm_event(PmEvent *event, void *user_data);
  static gboolean make_property_wrapped(const gchar *property_long_name,
                                        gint last_status,
                                        gint last_data1,
                                        void *user_data);
};

SWITCHER_DECLARE_PLUGIN(PortMidiSource);

}  // namespace switcher
#endif
