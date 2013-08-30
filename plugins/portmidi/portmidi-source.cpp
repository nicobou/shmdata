/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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

#include "portmidi-source.h"

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PortMidiSource,
				       "Midi (PortMidi)",
				       "midi source", 
				       "midi to shmdata and properties",
				       "LGPL",
				       "midisrc",
				       "Nicolas Bouillot");

  bool
  PortMidiSource::init ()
  {

    if (input_devices_enum_ [0].value_name == NULL)
      {
	g_debug ("no MIDI capture device detected");
	return false;
      }

    init_startable (this);
    
    custom_props_.reset (new CustomPropertyHelper ());
    devices_description_spec_ = 
      custom_props_->make_string_property ("devices-json", 
					   "Description of capture devices (json formated)",
					   get_devices_description_json ((PortMidi *)this),
					   (GParamFlags) G_PARAM_READABLE,
					   NULL,
					   get_devices_description_json,
					   (PortMidi *)this);
    
    register_property_by_pspec (custom_props_->get_gobject (), 
				devices_description_spec_, 
				"devices-json",
				"Capture Devices");
   
    device_ = input_devices_enum_[0].value;
    devices_enum_spec_ = 
      custom_props_->make_enum_property ("device", 
					 "Enumeration of MIDI capture devices",
					 device_, 
					 input_devices_enum_,
					 (GParamFlags) G_PARAM_READWRITE,
					 PortMidiSource::set_device,
					 PortMidiSource::get_device,
					 this);

    register_property_by_pspec (custom_props_->get_gobject (), 
				devices_enum_spec_, 
				"device",
				"Capture Device");
    
    midi_value_spec_ =
      custom_props_->make_int_property ("byte-rate", 
					"the byte rate (updated each second)",
					0,
					G_MAXINT,
					0,
					(GParamFlags) G_PARAM_READABLE,
					NULL,
					get_midi_value,
					this);
      
    
    // shmdata_writer_ = shmdata_any_writer_init ();
    
    // if (! shmdata_any_writer_set_path (shmdata_writer_, "/tmp/midi_truc"))
    // {
    //   g_debug ("**** The file exists, therefore a shmdata cannot be operated with this path.\n");
    //   shmdata_any_writer_close (shmdata_writer_);
    //   return false;
    // }
    // shmdata_any_writer_set_debug (shmdata_writer_, SHMDATA_ENABLE_DEBUG);
    // shmdata_any_writer_set_data_type (shmdata_writer_, "audio/midi");
    // shmdata_any_writer_start (shmdata_writer_);

    return true;
  }
  
  PortMidiSource::~PortMidiSource ()
  {
    //shmdata_any_writer_close (shmdata_writer_);
  }
  
  bool  
  PortMidiSource::start ()
  {
    unregister_property ("device");
    open_input_device(device_,
		      on_pm_event,
		      this);

    register_property_by_pspec (custom_props_->get_gobject (), 
				midi_value_spec_, 
				"raw-midi-value",
				"Raw Midi Value (4 bytes)");
    return true;
  }
  
  bool 
  PortMidiSource::stop ()
  {
    close_input_device (device_);
    unregister_property ("raw-midi-value");
    register_property_by_pspec (custom_props_->get_gobject (), 
				devices_enum_spec_, 
				"device",
				"Capture Device");
    return true;
  }

  void 
  PortMidiSource::set_device (const gint value, void *user_data)
  {
    PortMidiSource *context = static_cast <PortMidiSource *> (user_data);
    context->device_ = value;
  }
  
  gint 
  PortMidiSource::get_device (void *user_data)
  {
    PortMidiSource *context = static_cast <PortMidiSource *> (user_data);
    return context->device_;
  }

  void
  PortMidiSource::on_pm_event (PmEvent *event, void *user_data)
  {
    PortMidiSource *context = static_cast<PortMidiSource *> (user_data);
    
    context->midi_value_ = event->message;
    context->custom_props_->notify_property_changed (context->midi_value_spec_);

    // g_print ("from port midi  %u %u %u \n",
    // 	     Pm_MessageStatus(event->message),
    // 	     Pm_MessageData1(event->message),
    // 	     Pm_MessageData2(event->message));
  }
  
  gint
  PortMidiSource::get_midi_value (void *user_data)
  {
    PortMidiSource *context = static_cast<PortMidiSource *> (user_data);
    return context->midi_value_;
  }
  
}
