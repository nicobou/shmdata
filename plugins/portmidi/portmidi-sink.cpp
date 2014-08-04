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
    custom_props_ (new CustomPropertyHelper ()),
    devices_description_spec_ (nullptr),
    devices_enum_spec_ (nullptr),
    device_ (0)
  {}

  bool
  PortMidiSink::init ()
  {
    init_startable (this);
    init_segment (this);
    
    install_connect_method (std::bind (&PortMidiSink::connect, this, std::placeholders::_1),
			    nullptr,
			    nullptr,
			    std::bind (&PortMidiSink::can_sink_caps, this, std::placeholders::_1),
			    1);
    
    devices_description_spec_ = 
      custom_props_->make_string_property ("devices-json", 
					   "Description of capture devices (json formated)",
					   get_devices_description_json ((PortMidi *)this),
					   (GParamFlags) G_PARAM_READABLE,
					   nullptr,
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

    return true;
  }
  
  PortMidiSink::~PortMidiSink ()
  {}
  
  void
  PortMidiSink::on_shmreader_data (void *data,
				   int /*data_size*/,
				   unsigned long long /*timestamp*/,
				   const char */*type_description*/, 
				   void *user_data)
  {
    PmEvent *event = static_cast<PmEvent *> (data);
    push_midi_message (device_, 
		       Pm_MessageStatus(event->message),
		       Pm_MessageData1(event->message),
		       Pm_MessageData2(event->message));
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

  bool 
  PortMidiSink::connect (std::string path)
  {
    ShmdataAnyReader::ptr reader = std::make_shared<ShmdataAnyReader>();
    reader->set_data_type ("audio/midi");
    reader->set_absolute_timestamp (false);
    reader->set_path (path);
    reader->set_callback(std::bind (&PortMidiSink::on_shmreader_data,
				    this,
				    std::placeholders::_1,
				    std::placeholders::_2,
				    std::placeholders::_3,
				    std::placeholders::_4,
				    std::placeholders::_5),
			 NULL);
    reader->start ();
    register_shmdata (reader);
    return true;
  }
 
  bool
  PortMidiSink::can_sink_caps (std::string caps)
  {
    return 0 == caps.find ("audio/midi");
  }
}
