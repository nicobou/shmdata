/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher-pulse. 
 *
 * Partially from pactl.c Copyright 2004-2006 Lennart Poettering 
 *
 * switcher-pulse is free software; you can redistribute it and/or
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

#include "pulsesink.h"
#include "switcher/gst-utils.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PulseSink,
				       "Play To Audio Device (Pulse)",
				       "audio sink", 
				       "Inspecting Devices And Playing Audio To Outputs",
				       "LGPL",
				       "pulsesink",
				       "Nicolas Bouillot");
    
  PulseSink::PulseSink () :
    pulsesink_ (NULL),
    audioconvert_ (NULL),
    pulsesink_bin_ (NULL),
    connected_to_pulse_ (false),
    custom_props_ (new CustomPropertyHelper ()), 
    devices_description_spec_ (NULL),
    devices_description_ (NULL),
    device_name_spec_ (NULL),
    device_name_ (NULL),
    pa_glib_mainloop_ (NULL),
    pa_mainloop_api_ (NULL),
    pa_context_ (NULL),
    server_ (NULL),
    devices_ ()
  {}

  bool
  PulseSink::init_segment ()
  {
    device_name_ = g_strdup ("default");

    if (!make_elements ())
      return false;

    g_object_set (G_OBJECT (pulsesink_),"client", get_nick_name ().c_str (), NULL);

    pa_context_ = NULL;
    server_ = NULL;
    
    pa_glib_mainloop_ = pa_glib_mainloop_new(get_g_main_context ());
    
    pa_mainloop_api_ = pa_glib_mainloop_get_api(pa_glib_mainloop_);
    
    if (!(pa_context_ = pa_context_new(pa_mainloop_api_, NULL))) {
      g_debug ("PulseSink:: pa_context_new() failed.");
      return false;
    }
    
    pa_context_set_state_callback(pa_context_, pa_context_state_callback, this);
    
    if (pa_context_connect(pa_context_, server_, (pa_context_flags_t)0, NULL) < 0) {
      g_debug ("pa_context_connect() failed: %s", pa_strerror(pa_context_errno(pa_context_)));
      return false;
    }
    
    connected_to_pulse_ = true;
   
    devices_description_spec_ = custom_props_->make_string_property ("devices-json", 
      								     "Description of audio devices (json formated)",
      								     "default",
      								     (GParamFlags) G_PARAM_READABLE,
      								     NULL,
      								     PulseSink::get_devices_json,
     								     this);
    
    install_property_by_pspec (custom_props_->get_gobject (), 
     				devices_description_spec_, 
     				"devices-json",
     				"Devices Description");
    
    device_name_spec_ = custom_props_->make_string_property ("device-name", 
							     "Device pulse name",
							     "",
							     (GParamFlags) G_PARAM_READWRITE,
							     PulseSink::set_device_name,
							     PulseSink::get_device_name,
							     this);
    
    install_property_by_pspec (custom_props_->get_gobject (), 
				device_name_spec_, 
				"device-name",
				"Device Name");
    return true;
  }
  
  PulseSink::~PulseSink ()
  {
    if (connected_to_pulse_)
      {
	pa_context_disconnect (pa_context_);
	//pa_mainloop_api_->quit (pa_mainloop_api_, 0);
	pa_glib_mainloop_free(pa_glib_mainloop_);
      }
    if (devices_description_ != NULL)
      g_free (devices_description_);
    g_free (device_name_);
  }


  bool
  PulseSink::make_elements ()
  {

    if (!GstUtils::make_element ("pulsesink",&pulsesink_))
      return false;
    if (!GstUtils::make_element ("audioconvert",&audioconvert_))
      return false;
    if (!GstUtils::make_element ("bin",&pulsesink_bin_))
      return false;

    uninstall_property ("volume");
    uninstall_property ("mute");
    install_property (G_OBJECT (pulsesink_),"volume","volume", "Volume");
    install_property (G_OBJECT (pulsesink_),"mute","mute", "Mute");


    if (g_strcmp0 (device_name_, "default") != 0)
      g_object_set (G_OBJECT (pulsesink_), "device", device_name_, NULL);

    g_object_set (G_OBJECT (pulsesink_), "client", get_nick_name ().c_str (), NULL);

    gst_bin_add_many (GST_BIN (pulsesink_bin_),
      		      pulsesink_,
		      audioconvert_,
      		      NULL);

    gst_element_link (audioconvert_, pulsesink_);
    
    g_object_set (G_OBJECT (pulsesink_), "sync", FALSE, NULL);

    GstPad *sink_pad = gst_element_get_static_pad (audioconvert_, "sink");
    GstPad *ghost_sinkpad = gst_ghost_pad_new (NULL, sink_pad);
    gst_pad_set_active(ghost_sinkpad,TRUE);
    gst_element_add_pad (pulsesink_bin_, ghost_sinkpad); 
    gst_object_unref (sink_pad);
    set_sink_element (pulsesink_bin_);

    return true;
  }

  void 
  PulseSink::pa_context_state_callback(pa_context *pulse_context, void *user_data) {
    
    PulseSink *context = static_cast <PulseSink *> (user_data);
    
    switch (pa_context_get_state(pulse_context)) {
    case PA_CONTEXT_CONNECTING:
      //g_print ("PA_CONTEXT_CONNECTING\n");
      break;
    case PA_CONTEXT_AUTHORIZING:
      //g_print ("PA_CONTEXT_AUTHORIZING\n");
      break;
    case PA_CONTEXT_SETTING_NAME:
      //g_print ("PA_CONTEXT_SETTING_NAME\n");
      break;
    case PA_CONTEXT_READY: 
      //g_print ("PA_CONTEXT_READY\n");
      context->make_device_description (pulse_context);
      
      pa_context_set_subscribe_callback (pulse_context,
					 on_pa_event_callback,
					 NULL); 	
      
      pa_operation_unref(pa_context_subscribe (pulse_context,
					       (pa_subscription_mask_t) (PA_SUBSCRIPTION_MASK_SINK|
									 PA_SUBSCRIPTION_MASK_SOURCE|
									 PA_SUBSCRIPTION_MASK_SINK_INPUT|
									 PA_SUBSCRIPTION_MASK_SOURCE_OUTPUT|
									 PA_SUBSCRIPTION_MASK_MODULE|
									 PA_SUBSCRIPTION_MASK_CLIENT|
									 PA_SUBSCRIPTION_MASK_SAMPLE_CACHE|
									 PA_SUBSCRIPTION_MASK_SERVER|
									 PA_SUBSCRIPTION_MASK_CARD),
					       NULL, //pa_context_success_cb_t cb,
					       NULL) //void *userdata);
			 );
      
      break;
    case PA_CONTEXT_TERMINATED:
      g_debug ("PulseSink: PA_CONTEXT_TERMINATED\n");
      pa_context_unref(context->pa_context_);
      context->pa_context_ = NULL;
      break;
    case PA_CONTEXT_FAILED:
      //g_print ("PA_CONTEXT_FAILED\n");
      break;
    default:
      g_debug ("PulseSink Context error: %s\n",pa_strerror(pa_context_errno(pulse_context)));
    }
  }

  void 
  PulseSink::make_json_description ()
  {
    if (devices_description_ != NULL)
      g_free (devices_description_);
    
    JSONBuilder::ptr builder (new JSONBuilder ());
    builder->reset();
    builder->begin_object ();
    builder->set_member_name ("devices");
    builder->begin_array ();

    for (auto &it: devices_)
      {
	builder->begin_object ();
	builder->add_string_member ("long name", it.second.description_.c_str());
	builder->add_string_member ("name", it.second.name_.c_str());
	builder->add_string_member ("state", it.second.state_.c_str());
	builder->add_string_member ("sample format", it.second.sample_format_.c_str());
	builder->add_string_member ("sample rate", it.second.sample_rate_.c_str());
	builder->add_string_member ("channels", it.second.channels_.c_str());
	builder->add_string_member ("active port", it.second.active_port_.c_str());
	builder->end_object ();
      }
    
    builder->end_array ();
    builder->end_object ();
    devices_description_ = g_strdup (builder->get_string (true).c_str ());
    //g_print ("%s\n",devices_description_);
    GObjectWrapper::notify_property_changed (gobject_->get_gobject (), devices_description_spec_);
  }
  
  void 
  PulseSink::get_sink_info_callback (pa_context *pulse_context, 
				     const pa_sink_info *i, 
				     int is_last, 
				     void *user_data) 
  {
    PulseSink *context = static_cast <PulseSink *> (user_data);
    
    if (is_last < 0) {
      g_debug ("Failed to get sink information: %s", pa_strerror(pa_context_errno(pulse_context)));
      return;
    }
    
    if (is_last) {
      pa_operation *operation = pa_context_drain(pulse_context, NULL, NULL);
      if (operation)
        pa_operation_unref(operation);
      
      context->make_json_description ();
      return;
    }
    
    DeviceDescription description;
    switch (i->state) {
     case PA_SINK_INIT:
       description.state_ = "INIT";
       g_print ("state: INIT \n");
       break;
     case PA_SINK_UNLINKED:
       description.state_ = "UNLINKED";
       //g_print ("state: UNLINKED \n");
       break;
     case PA_SINK_INVALID_STATE:
       description.state_ = "n/a";      
       //g_print ("state: n/a \n");
       break;
     case PA_SINK_RUNNING:
       description.state_ = "RUNNING";      
       //g_print ("state: RUNNING \n");
       break;
     case PA_SINK_IDLE:
       description.state_ = "IDLE";      
       //g_print ("state: IDLE \n");
       break;
     case PA_SINK_SUSPENDED:
       description.state_ = "SUSPENDED";      
       //g_print ("state: SUSPENDED \n");
       break;
     }

     description.name_ = i->name;
     if (i->description == NULL)
       description.description_ = "";
     else
       description.description_ = i->description;
    
     description.sample_format_ = pa_sample_format_to_string (i->sample_spec.format);
     gchar *rate = g_strdup_printf ("%u", i->sample_spec.rate);
     description.sample_rate_ = rate;
     g_free (rate);
     gchar *channels = g_strdup_printf ("%u", i->sample_spec.channels);
     description.channels_ = channels;
     g_free (channels);
    
     // g_print ("Name: %s\n"
     // 	     "Description: %s\n"
     // 	     " format: %s\n"
     // 	     " rate: %u\n"
     // 	     " channels: %u\n",
     // 	     //"Channel Map: %s\n",
     // 	     i->name,
     // 	     i->description,//warning this can be NULL
     // 	     pa_sample_format_to_string (i->sample_spec.format),
     // 	     i->sample_spec.rate,
     // 	     i->sample_spec.channels//,
     // 	     //pa_channel_map_snprint(cm, sizeof(cm), &i->channel_map)
     // 	     );

     if (i->ports) {
         pa_sink_port_info **p;
         //printf("\tPorts:\n");
         for (p = i->ports; *p; p++)
     	  {
     	    //printf("\t\t%s: %s (priority. %u)\n", (*p)->name, (*p)->description, (*p)->priority);
     	    description.ports_.push_back (std::make_pair ((*p)->name, (*p)->description));
     	  }
     }

     if (i->active_port)
       {
     	//printf("\tActive Port: %s\n", i->active_port->name);
     	description.active_port_ = i->active_port->description;
       }
     else
       description.active_port_ = "n/a";

    context->devices_[description.name_] = description;
    
     // if (i->formats) {
     //   uint8_t j;
     //   printf("\tFormats:\n");
     //   for (j = 0; j < i->n_formats; j++)
     // 	printf("\t\t%s\n", pa_format_info_snprint(f, sizeof(f), i->formats[j]));
     // }
  }

  void 
  PulseSink::make_device_description (pa_context *pulse_context)
  {
    devices_.clear ();
    pa_operation_unref(pa_context_get_sink_info_list(pulse_context, get_sink_info_callback, this));
  }


  void 
    PulseSink::on_pa_event_callback (pa_context *pulse_context, 
				    pa_subscription_event_type_t pulse_event_type,
				     uint32_t /*index*/, 
				    void *user_data)
  {
    PulseSink *context = static_cast<PulseSink *> (user_data);

    if ((pulse_event_type & PA_SUBSCRIPTION_EVENT_FACILITY_MASK) == PA_SUBSCRIPTION_EVENT_SINK) 
      {
	if ((pulse_event_type & PA_SUBSCRIPTION_EVENT_TYPE_MASK) == PA_SUBSCRIPTION_EVENT_NEW) 
	  {
	    context->make_device_description (pulse_context);
	    return;
	  }
      }
    
    if ((pulse_event_type & PA_SUBSCRIPTION_EVENT_FACILITY_MASK) == PA_SUBSCRIPTION_EVENT_SINK) 
      {
	if ((pulse_event_type & PA_SUBSCRIPTION_EVENT_TYPE_MASK) == PA_SUBSCRIPTION_EVENT_REMOVE) 
	  {
	    context->make_device_description (pulse_context);
	    return;
	  }
      }
  }
  
  gchar *
  PulseSink::get_devices_json (void *user_data)
  {
    PulseSink *context = static_cast<PulseSink *> (user_data);
    if (context->devices_description_ == NULL)
      context->devices_description_ = g_strdup ("{ \"devices\" : [] }");

    return context->devices_description_;
  }

  gchar *
  PulseSink::get_device_name (void *user_data)
  {
    PulseSink *context = static_cast<PulseSink *> (user_data);
    return context->device_name_;
  }
 
  void 
  PulseSink::set_device_name (const gchar *value, 
			      void *user_data)
  {
    PulseSink *context = static_cast<PulseSink *> (user_data);
    g_free (context->device_name_);
    context->device_name_ = g_strdup (value);
    GObjectWrapper::notify_property_changed (context->gobject_->get_gobject (), 
					     context->device_name_spec_);
    context->make_elements ();
  }

  void 
  PulseSink::on_shmdata_disconnect () 
  {
    reset_bin ();
  }

  void 
  PulseSink::on_shmdata_connect (std::string /* shmdata_sochet_path */) 
  {
    make_elements ();
  }
}//end of PulseSink class
  
  
  
