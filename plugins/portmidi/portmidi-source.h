/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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

#include "portmidi-devices.h"
#include "switcher/quiddity.h"
#include "switcher/startable-quiddity.h"
#include "switcher/custom-property-helper.h"
#include "shmdata/any-data-writer.h"

namespace switcher
{
  
  class PortMidiSource : public Quiddity, public StartableQuiddity, public PortMidi
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PortMidiSource);
    ~PortMidiSource ();

    bool start ();
    bool stop ();
    
  private:
    //shmdata_any_writer_t *shmdata_writer_;
    
    //properties
    CustomPropertyHelper::ptr custom_props_;
    GParamSpec *devices_description_spec_;

    //device selection
    GParamSpec *devices_enum_spec_;
    gint device_;
    static void set_device (const gint value, void *user_data);
    static gint get_device (void *user_data);

    //raw midi value
    //4 bytes, as PmMessage decribed in portmidi.h
    GParamSpec *midi_value_spec_;
    gint midi_value_;
    static gint get_midi_value (void *user_data);
    
    static void on_pm_event (PmEvent *event, void *user_data);
    
  };
  
  SWITCHER_DECLARE_PLUGIN(PortMidiSource);
  
}  // end of namespace

#endif // ifndef
