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

#include "portmidi-sink.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PortMidiSink,
				       "Midi (PortMidiSink)",
				       "midi sink", 
				       "shmdata to midi",
				       "LGPL",
				       "midisink",				
				       "Nicolas Bouillot");
  PortMidiSink::PortMidiSink () :
    reader_ (shmdata_any_reader_init ()),
    custom_props_ (new CustomPropertyHelper ()),
    devices_description_spec_ (NULL),
    shmdata_path_spec_ (NULL),
    shmdata_path_ (g_strdup ("")),
    devices_enum_spec_ (NULL),
    device_ (0)
  {}

  bool
  PortMidiSink::init ()
  {
    init_startable (this);
    devices_description_spec_ = 
      custom_props_->make_string_property ("devices-json", 
					   "Description of capture devices (json formated)",
					   get_devices_description_json ((PortMidi *)this),
					   (GParamFlags) G_PARAM_READABLE,
					   NULL,
					   get_devices_description_json,
					   (PortMidi *)this);
    
    install_property_by_pspec (custom_props_->get_gobject (), 
				devices_description_spec_, 
				"devices-json",
				"Capture Devices");

    device_ = output_devices_enum_[0].value;
    devices_enum_spec_ = 
      custom_props_->make_enum_property ("device", 
					 "Enumeration of MIDI capture devices",
					 device_, 
					 output_devices_enum_,
					 (GParamFlags) G_PARAM_READWRITE,
					 PortMidiSink::set_device,
					 PortMidiSink::get_device,
					 this);
    install_property_by_pspec (custom_props_->get_gobject (), 
				devices_enum_spec_, 
				"device",
				"Capture Device");

    shmdata_path_spec_ = 
      custom_props_->make_string_property ("shmdata-path", 
					   "path of the shmdata to connect with",
					   shmdata_path_,
					   (GParamFlags) G_PARAM_READWRITE,
					   PortMidiSink::set_shmdata_path,
					   PortMidiSink::get_shmdata_path,
					   this);
    
    install_property_by_pspec (custom_props_->get_gobject (), 
				shmdata_path_spec_, 
				"shmdata-path",
				"Shmdata Path");

    shmdata_any_reader_set_debug (reader_, SHMDATA_ENABLE_DEBUG);
    shmdata_any_reader_set_on_data_handler (reader_, 
					    &on_shmreader_data,
					    this);
    shmdata_any_reader_set_data_type (reader_, "audio/midi");
    //shmdata_any_reader_set_absolute_timestamp (reader, SHMDATA_DISABLE_ABSOLUTE_TIMESTAMP);

    return true;
  }
  
  PortMidiSink::~PortMidiSink ()
  {
    g_free (shmdata_path_);
    shmdata_any_reader_close (reader_);
  }
  
  void
  PortMidiSink::on_shmreader_data (shmdata_any_reader_t */*reader*/,
				   void *shmbuf,
				   void *data,
				   int /*data_size*/,
				   unsigned long long /*timestamp*/,
				   const char */*type_description*/, 
				   void *user_data)
  {
    PortMidiSink *context = static_cast <PortMidiSink *> (user_data);

    // printf ("data %p, data size %d, timestamp %llu, type descr %s\n",
    //  	    data, data_size, timestamp, type_description);

    PmEvent *event = (PmEvent *)data;
    
    context->push_midi_message (context->device_, 
				Pm_MessageStatus(event->message),
				Pm_MessageData1(event->message),
				Pm_MessageData2(event->message));
    
    // g_print ("from shm: %u %u %u \n",
    // 	     Pm_MessageStatus(event->message),
    // 	     Pm_MessageData1(event->message),
    // 	     Pm_MessageData2(event->message));

    //free the data, can also be called later
    shmdata_any_reader_free (shmbuf);
  }


  void 
  PortMidiSink::set_device (const gint value, void *user_data)
  {
    PortMidiSink *context = static_cast <PortMidiSink *> (user_data);
    context->device_ = value;
  }
 
  gint 
  PortMidiSink::get_device (void *user_data)
  {
    PortMidiSink *context = static_cast <PortMidiSink *> (user_data);
    return context->device_;
  }

  bool  
  PortMidiSink::start ()
  {
    uninstall_property ("device");
    open_output_device (device_);
    gint stat = 165;
    gint data1 = 1;
    gint data2 = 67;
    push_midi_message (device_, (unsigned char) stat, (unsigned char)data1, (unsigned char)data2);
    return true;
  }

  bool 
  PortMidiSink::stop ()
  {
    close_output_device (device_);
    install_property_by_pspec (custom_props_->get_gobject (), 
     				devices_enum_spec_, 
     				"device",
     				"Capture Device");
    return true;
  }

  void 
  PortMidiSink::set_shmdata_path (const gchar * value, void *user_data)
  {
    PortMidiSink *context = static_cast <PortMidiSink *> (user_data);
    g_free (context->shmdata_path_);
    context->shmdata_path_ = g_strdup (value);
    shmdata_any_reader_start (context->reader_, context->shmdata_path_);
  }
 
  const gchar *
  PortMidiSink::get_shmdata_path (void *user_data)
  {
    PortMidiSink *context = static_cast <PortMidiSink *> (user_data);
    return context->shmdata_path_;
  }
  

}
