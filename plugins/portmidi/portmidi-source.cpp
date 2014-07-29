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

  PortMidiSource::PortMidiSource () :
    shm_any_ (std::make_shared<ShmdataAnyWriter> ()),
    last_status_ (-1),
    last_data1_ (-1),
    last_data2_ (-1),
    custom_props_ (new CustomPropertyHelper ()),
    devices_description_spec_ (nullptr),
    devices_enum_spec_ (nullptr),
    device_ (0),
    midi_value_spec_ (nullptr),
    make_property_for_next_midi_event_ (FALSE),
    next_property_name_ (),
    prop_specs_ (),
    midi_property_contexts_ (),
    midi_channels_ (),
    midi_values_ (), 
    unused_props_specs_ ()
  {}

  PortMidiSource::~PortMidiSource ()
  {}
  
  
  bool
  PortMidiSource::init ()
  {

    if (input_devices_enum_ [0].value_name == nullptr)
      {
	g_debug ("no MIDI capture device detected");
	return false;
      }
    init_startable (this);
    init_segment (this);
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

    install_property_by_pspec (custom_props_->get_gobject (), 
				devices_enum_spec_, 
				"device",
				"Capture Device");
    

     midi_value_spec_ =
      custom_props_->make_int_property ("last-midi-value", 
					"the last midi value",
					0,
					127,
					0,
					(GParamFlags) G_PARAM_READABLE,
					nullptr,
					get_midi_value,
					this);

    install_method ("Next MIDI Event To Property", //long name
		    "next_midi_event_to_property", //name
		    "Wait for a MIDI event and make a property for this channel", //description
		    "success or fail", //return description
		    Method::make_arg_description ("Property Long Name", //first arg long name
						  "property_long_name", //fisrt arg name
						  "string", //first arg description 
						  nullptr),
  		    (Method::method_ptr) &next_midi_event_to_property_method, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, nullptr),
		    this);

    install_method ("Last MIDI Event To Property", //long name
		    "last_midi_event_to_property", //name
		    "make a property with the given name from the next incoming MIDI event", //description
		    "success or fail", //return description
		    Method::make_arg_description ("Property Long Name", //first arg long name
						  "property_long_name", //fisrt arg name
						  "string", //first arg description 
						  nullptr),
  		    (Method::method_ptr) &last_midi_event_to_property_method, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, nullptr),
		    this);

    install_method ("Remove Midi Property", //long name
		    "remove_midi_property", //name
		    "remove a property made with Make Property", //description
		    "success or fail", //return description
		    Method::make_arg_description ("Property Long Name", //first arg long name
						  "property_long_name", //fisrt arg name
						  "string", //first arg description 
						  nullptr),
  		    (Method::method_ptr) &remove_property_method, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, nullptr),
		    this);

    std::string shm_any_name = make_file_name ("midi");
    shm_any_->set_path (shm_any_name.c_str());
    g_message ("%s created a new shmdata any writer (%s)", 
	       get_nick_name ().c_str(), 
	       shm_any_name.c_str ());
    shm_any_->set_data_type ("audio/midi");
    shm_any_->start ();
    register_shmdata (shm_any_);

    return true;
  }
  
  bool  
  PortMidiSource::start ()
  {
    uninstall_property ("device");
    open_input_device(device_,
		      on_pm_event,
		      this);
    install_property_by_pspec (custom_props_->get_gobject (), 
				midi_value_spec_, 
				"last-midi-value",
				"Last Midi Value");
    return true;
  }
  
  bool 
  PortMidiSource::stop ()
  {
    close_input_device (device_);
    uninstall_property ("last-midi-value");
    install_property_by_pspec (custom_props_->get_gobject (), 
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
    


    PmEvent *tmp_event = (PmEvent *)g_malloc (sizeof (PmEvent));
    tmp_event->message = event->message;
    tmp_event->timestamp = event->timestamp;
    context->shm_any_->push_data (tmp_event,
     				  sizeof (PmEvent),
				  //(timestamp is in ms)
				  (unsigned long long) tmp_event->timestamp * 1000000,
				  g_free, 
				  tmp_event);

    
    guint status = Pm_MessageStatus(event->message);
    guint data1 = Pm_MessageData1(event->message);
    guint data2 = Pm_MessageData2(event->message);
    
    context->last_status_ = (gint) status;
    context->last_data1_ = (gint) data1;
    context->last_data2_ = (gint) data2;
    context->custom_props_->notify_property_changed (context->midi_value_spec_);

    // g_print ("to shm:  %u %u %u event ts %d tmp_event_ts %d\n",
    //   	     status,
    //   	     data1,
    //   	     data2,
    // 	     event->timestamp);

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
      if (context->make_property (context->next_property_name_))
	context->make_property_for_next_midi_event_ = FALSE;
  }
  
  gboolean
  PortMidiSource::next_midi_event_to_property_method (gchar *long_name, void *user_data)
  {
    PortMidiSource *context = static_cast<PortMidiSource *> (user_data);
    context->make_property_for_next_midi_event_ = TRUE;  
    context->next_property_name_ = long_name;
     
    timespec delay;
    delay.tv_sec = 0;
    delay.tv_nsec = 1000000; //1ms
    while (context->make_property_for_next_midi_event_)
      nanosleep(&delay, nullptr);
    return TRUE;
  }

  gboolean
  PortMidiSource::last_midi_event_to_property_method (gchar *long_name, void *user_data)
  {
    PortMidiSource *context = static_cast<PortMidiSource *> (user_data);
    if (!context->make_property (long_name))
      return FALSE;

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
    
    gchar *prop_name = g_strdup_printf ("%u-%u", midi_channel.first, midi_channel.second);
    context->uninstall_property (prop_name);
    context->unused_props_specs_[prop_name] = context->prop_specs_[long_name];
    context->prop_specs_.erase (long_name);
    context->midi_channels_.erase (midi_channel);
    context->midi_values_.erase (long_name);
    g_free (prop_name);
    return TRUE;
  }

  bool
  PortMidiSource::make_property (std::string property_long_name)
  {
    if (last_status_ == -1 || last_data1_ == -1)
      {
	g_debug("portmidisource cannot make a property without midi event");
	return false;
      }

    if (midi_channels_.find (std::make_pair (last_status_, last_data1_)) 
	!= midi_channels_.end ())
      {
	g_debug ("Midi Channels %u %u is already a property (is currently named %s)",
		 last_status_,
		 last_data1_,
		 midi_channels_.find (std::make_pair (last_status_, last_data1_))->second.c_str ());
	return false;
      }
    
    midi_channels_[std::make_pair (last_status_, last_data1_)] = property_long_name;
    gchar *prop_name = g_strdup_printf ("%u-%u",
					last_status_,
					last_data1_);
    midi_values_ [property_long_name] = last_data2_;
    
    if (unused_props_specs_.find (prop_name) == unused_props_specs_.end ())
      {
	MidiPropertyContext midi_property_context;
	midi_property_context.port_midi_source_ = this;
	midi_property_context.property_long_name_ = property_long_name;
	midi_property_contexts_[property_long_name] = midi_property_context;
	
	prop_specs_[property_long_name] = 
	  custom_props_->make_int_property (prop_name, 
					    "midi value",
					    0,
					    127,
					    0,
					    (GParamFlags) G_PARAM_READABLE,
					    nullptr,
					    get_midi_property_value,
					    &midi_property_contexts_[property_long_name]);
      }
    else
      {
	prop_specs_[property_long_name] = unused_props_specs_[prop_name];
	unused_props_specs_.erase (prop_name);
      }
    
    install_property_by_pspec (custom_props_->get_gobject (), 
				prop_specs_[property_long_name], 
				prop_name,
				property_long_name.c_str ());
    g_free (prop_name);
    
    
    return true;
  }


  gint
  PortMidiSource::get_midi_value (void *user_data)
  {
    PortMidiSource *context = static_cast<PortMidiSource *> (user_data);
    return context->last_data2_;
  }


}
