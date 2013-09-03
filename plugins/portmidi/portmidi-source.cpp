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
#include <time.h>

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
    make_property_for_next_midi_event_ = FALSE;

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
      custom_props_->make_int_property ("raw-midi-value", 
					"the raw midi value (4 bytes)",
					0,
					G_MAXINT,
					0,
					(GParamFlags) G_PARAM_READABLE,
					NULL,
					get_midi_value,
					this);
      

    
    publish_method ("Make MIDI property", //long name
		    "make_midi_property", //name
		    "Wait for a MIDI event and make a property for this channel", //description
		    "success or fail", //return description
		    Method::make_arg_description ("Property Long Name", //first arg long name
						  "property_long_name", //fisrt arg name
						  "string", //first arg description 
						  NULL),
  		    (Method::method_ptr) &make_property_method, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    this);

    publish_method ("Remove Midi Property", //long name
		    "remove_midi_property", //name
		    "remove a property made with Make Property", //description
		    "success or fail", //return description
		    Method::make_arg_description ("Property Long Name", //first arg long name
						  "property_long_name", //fisrt arg name
						  "string", //first arg description 
						  NULL),
  		    (Method::method_ptr) &remove_property_method, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
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

    guint status = Pm_MessageStatus(event->message);
    guint data1 = Pm_MessageData1(event->message);
    guint data2 = Pm_MessageData2(event->message);

    // g_print ("from port midi  %u %u %u \n",
    //  	     status,
    //  	     data1,
    //  	     data2);

    //updating property if needed
    if (context->midi_channels_.find (std::make_pair (status, data1)) != context->midi_channels_.end ())
      {
	std::string prop_long_name = 
	  context->midi_channels_[std::make_pair (status, data1)];
	context->midi_values_[prop_long_name] = data2;
	context->custom_props_->notify_property_changed (context->prop_specs_[prop_long_name]);
      }
    
    //making property if needed
    if (context->make_property_for_next_midi_event_)
      {
	if (context->midi_channels_.find (std::make_pair (status, data1)) != context->midi_channels_.end ())
	  {
	    g_debug ("Midi Channels %u %u is already a property (is currently named %s)",
		     status,
		     data1,
		     context->midi_channels_.find (std::make_pair (status, data1))->second.c_str ());
	    return;
	  }
	context->midi_channels_[std::make_pair (status, data1)] = context->next_property_name_;
	gchar *prop_name = g_strdup_printf ("%u_%u",
					    status,
					    data1);
	context->midi_values_ [context->next_property_name_] = data2;

	if (context->unused_props_specs_.find (prop_name) == context->unused_props_specs_.end ())
	  {
	    MidiPropertyContext midi_property_context;
	    midi_property_context.port_midi_source_ = context;
	    midi_property_context.property_long_name_ = context->next_property_name_;
	    context->midi_property_contexts_[context->next_property_name_] = midi_property_context;
	    
	    context->prop_specs_[context->next_property_name_] = 
	      context->custom_props_->make_int_property (prop_name, 
							 "midi value",
							 0,
							 127,
							 0,
							 (GParamFlags) G_PARAM_READABLE,
							 NULL,
							 get_midi_property_value,
							 &context->midi_property_contexts_[context->next_property_name_]);
	  }
	else
	  {
	    context->prop_specs_[context->next_property_name_] = context->unused_props_specs_[prop_name];
	    context->unused_props_specs_.erase (prop_name);
	  }
	
	context->register_property_by_pspec (context->custom_props_->get_gobject (), 
					     context->prop_specs_[context->next_property_name_], 
					     prop_name,
					     context->next_property_name_.c_str ());
	g_free (prop_name);
	context->make_property_for_next_midi_event_ = FALSE;
      }
  }
  
  gint
  PortMidiSource::get_midi_value (void *user_data)
  {
    PortMidiSource *context = static_cast<PortMidiSource *> (user_data);
    return context->midi_value_;
  }

  gboolean
  PortMidiSource::make_property_method (gchar *long_name, void *user_data)
  {
     PortMidiSource *context = static_cast<PortMidiSource *> (user_data);
     context->make_property_for_next_midi_event_ = TRUE;  
     context->next_property_name_ = long_name;
     
     timespec delay;
     delay.tv_sec = 0;
     delay.tv_nsec = 1000000; //1ms
     while (context->make_property_for_next_midi_event_)
       nanosleep(&delay, NULL);
    return TRUE;
  }

  gint 
  PortMidiSource::get_midi_property_value (void *user_data)
  {
    MidiPropertyContext *context = static_cast <MidiPropertyContext *> (user_data);
    return context->port_midi_source_->midi_values_[context->property_long_name_];
  }


  gboolean
  PortMidiSource::remove_property_method (gchar *long_name, void *user_data)
  {
    PortMidiSource *context = static_cast<PortMidiSource *> (user_data);

    if (context->midi_property_contexts_.find (long_name) == context->midi_property_contexts_.end ())
      {
	g_debug ("property %s not found for removing",
		 long_name);
	return FALSE;
      }
    
    std::pair <guint, guint> midi_channel;
    for (auto &it: context->midi_channels_)
      {
	if (g_strcmp0 (it.second.c_str (), long_name) == 0)
	  {
	    midi_channel = it.first;
	    break;
	  }
      }
    
    gchar *prop_name = g_strdup_printf ("%u_%u", midi_channel.first, midi_channel.second);
    context->unregister_property (prop_name);
    context->unused_props_specs_[prop_name] = context->prop_specs_[long_name];
    context->prop_specs_.erase (long_name);
    context->midi_channels_.erase (midi_channel);
    context->midi_values_.erase (long_name);
    g_free (prop_name);
    return TRUE;
  }
}
