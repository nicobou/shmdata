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

#include "pulsesrc.h"
#include "switcher/gst-utils.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PulseSrc,
				       "Audio Device Source (Pulse)",
				       "audio source", 
				       "Inspecting Devices And Getting Audio From Inputs",
				       "LGPL",
				       "pulsesrc",
				       "Nicolas Bouillot");
    
  bool
  PulseSrc::init ()
  {
    if (!make_elements ())
      return false;
    //set the name before registering properties
    set_name (gst_element_get_name (pulsesrc_));

    pa_context_ = NULL;
    server_ = NULL;

    pa_glib_mainloop_ = pa_glib_mainloop_new(get_g_main_context ());
    
    pa_mainloop_api_ = pa_glib_mainloop_get_api(pa_glib_mainloop_);
    
    if (!(pa_context_ = pa_context_new(pa_mainloop_api_, NULL))) {
      g_debug ("PulseSrc:: pa_context_new() failed.");
      return false;
    }
    
    pa_context_set_state_callback(pa_context_, pa_context_state_callback, this);

    if (pa_context_connect(pa_context_, server_, (pa_context_flags_t)0, NULL) < 0) {
      g_debug ("pa_context_connect() failed: %s", pa_strerror(pa_context_errno(pa_context_)));
      return false;
    }


    capture_devices_description_ = NULL;
    register_method("capture",
		    (void *)&capture_wrapped, 
		    Method::make_arg_type_description (G_TYPE_NONE, 
						       NULL),
		    (gpointer)this);
    set_method_description ("capture", 
			    "start capturing from default device", 
			    Method::make_arg_description ("none",
 							  NULL));


    register_method("capture_device",
		    (void *)&capture_device_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, 
						       NULL),
		    (gpointer)this);
    set_method_description ("capture_device", 
			    "start capturing from selected device", 
			    Method::make_arg_description ("pulse_name_device",
							  "Pulse Audio Device Name",
							  NULL));


    custom_props_.reset (new CustomPropertyHelper ());
    capture_devices_description_spec_ = custom_props_->make_string_property ("capture-devices-json", 
									     "Description of capture devices (json formated)",
									     "",
									     (GParamFlags) G_PARAM_READABLE,
									     NULL,
									     PulseSrc::get_capture_devices_json,
									     this);
    
    register_property_by_pspec (custom_props_->get_gobject (), 
				capture_devices_description_spec_, 
				"capture-devices-json",
				"Capture Devices");
    return true;
  }

  PulseSrc::~PulseSrc ()
  {
    pa_context_disconnect (pa_context_);
    //pa_mainloop_api_->quit (pa_mainloop_api_, 0);
    pa_glib_mainloop_free(pa_glib_mainloop_);

    if (capture_devices_description_ != NULL)
      g_free (capture_devices_description_);
    clean_elements ();
  }


  bool
  PulseSrc::make_elements ()
  {
    clean_elements ();

    if (!GstUtils::make_element ("pulsesrc",&pulsesrc_))
      return false;
    if (!GstUtils::make_element ("capsfilter",&capsfilter_))
      return false;
    if (!GstUtils::make_element ("bin",&pulsesrc_bin_))
      return false;
    
    gst_bin_add_many (GST_BIN (pulsesrc_bin_),
		      pulsesrc_,
		      capsfilter_,
		      NULL);

    gst_element_link (pulsesrc_, capsfilter_);

    GstPad *src_pad = gst_element_get_static_pad (capsfilter_, "src");
    GstPad *ghost_srcpad = gst_ghost_pad_new (NULL, src_pad);
    gst_pad_set_active(ghost_srcpad,TRUE);
    gst_element_add_pad (pulsesrc_bin_, ghost_srcpad); 
    gst_object_unref (src_pad);
    return true;
  }

  void
  PulseSrc::clean_elements ()
  {
    GstUtils::clean_element (pulsesrc_);
    //GstUtils::clean_element (capsfilter_);//FIXME
    GstUtils::clean_element (pulsesrc_bin_);
  }

  void 
  PulseSrc::pa_context_state_callback(pa_context *pulse_context, void *user_data) {
    
    PulseSrc *context = static_cast <PulseSrc *> (user_data);
    
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
      pa_operation_unref(pa_context_get_source_info_list(pulse_context,
							 get_source_info_callback, 
							 NULL));
      
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
      g_debug ("PulseSrc: PA_CONTEXT_TERMINATED\n");
      pa_context_unref(context->pa_context_);
      context->pa_context_ = NULL;
      break;
    case PA_CONTEXT_FAILED:
      //g_print ("PA_CONTEXT_FAILED\n");
      break;
    default:
      g_debug ("PulseSrc Context error: %s\n",pa_strerror(pa_context_errno(pulse_context)));
    }

  }

  void 
  PulseSrc::get_source_info_callback(pa_context *pulse_context, const pa_source_info *i, int is_last, void *userdata) {

    if (is_last < 0) {
      g_debug ("Failed to get source information: %s", pa_strerror(pa_context_errno(pulse_context)));
      return;
    }
    
    if (is_last) {
      pa_operation *operation = pa_context_drain(pulse_context, NULL, NULL);
      if (operation)
        pa_operation_unref(operation);
      return;
    }
    
    //HERE make json
    printf(":: source :: \n");
    
    switch (i->state) {
    case PA_SOURCE_INIT:
      g_print ("state: INIT \n");
      break;
    case PA_SOURCE_UNLINKED:
      g_print ("state: UNLINKED \n");
      break;
    case PA_SOURCE_INVALID_STATE:
      g_print ("state: n/a \n");
      break;
    case PA_SOURCE_RUNNING:
      g_print ("state: RUNNING \n");
      break;
    case PA_SOURCE_IDLE:
      g_print ("state: IDLE \n");
      break;
    case PA_SOURCE_SUSPENDED:
      g_print ("state: SUSPENDED \n");
      break;
    }


    g_print ("\tName: %s\n"
	     "\tDescription: %s\n"
	     "\t format: %s\n"
	     "\t rate: %u\n"
	     "\t channels: %u\n",
	     //"\tChannel Map: %s\n",
	     i->name,
	     i->description,//warning this can be NULL
	     pa_sample_format_to_string (i->sample_spec.format),
	     i->sample_spec.rate,
	     i->sample_spec.channels//,
	     //pa_channel_map_snprint(cm, sizeof(cm), &i->channel_map)
	     );
    

    if (i->ports) {
        pa_source_port_info **p;

        printf("\tPorts:\n");
        for (p = i->ports; *p; p++)
            printf("\t\t%s: %s (priority. %u)\n", (*p)->name, (*p)->description, (*p)->priority);
    }

    if (i->active_port)
        printf("\tActive Port: %s\n",
               i->active_port->name);

    // if (i->formats) {
    //   uint8_t j;
    
    //   printf("\tFormats:\n");
    //   for (j = 0; j < i->n_formats; j++)
    // 	printf("\t\t%s\n", pa_format_info_snprint(f, sizeof(f), i->formats[j]));
    // }
  }

  void 
    PulseSrc::on_pa_event_callback (pa_context *c, 
				    pa_subscription_event_type_t t,
				    uint32_t idx, 
				    void *userdata)
  {
    if ((t & PA_SUBSCRIPTION_EVENT_FACILITY_MASK) == PA_SUBSCRIPTION_EVENT_SOURCE) {
      if ((t & PA_SUBSCRIPTION_EVENT_TYPE_MASK) == PA_SUBSCRIPTION_EVENT_NEW) {
	pa_operation_unref(pa_context_get_source_info_list(c, get_source_info_callback, NULL));
	return;
      }
    }
    
    if ((t & PA_SUBSCRIPTION_EVENT_FACILITY_MASK) == PA_SUBSCRIPTION_EVENT_SOURCE) {
      if ((t & PA_SUBSCRIPTION_EVENT_TYPE_MASK) == PA_SUBSCRIPTION_EVENT_REMOVE) {
	pa_operation_unref(pa_context_get_source_info_list(c, get_source_info_callback, NULL));
	return;
      }
    }
  }

  gchar *
  PulseSrc::get_capture_devices_json (void *user_data)
  {
    PulseSrc *context = static_cast<PulseSrc *> (user_data);
    if (context->capture_devices_description_ != NULL)
      g_free (context->capture_devices_description_);

    

    //context->capture_devices_description_ = g_strdup (builder->get_string (true).c_str ());
    context->capture_devices_description_ = g_strdup ("{error: \"todo\"}");
    return context->capture_devices_description_;
  }

  gboolean 
  PulseSrc::capture_wrapped (gpointer device_file_path, 
			    gpointer user_data)
  {
    PulseSrc *context = static_cast<PulseSrc *>(user_data);
    
    if (context->capture_device ("NONE"))
      return TRUE;
    else
      return FALSE;
  }

    gboolean 
    PulseSrc::capture_device_wrapped (gpointer pulse_device_name,
				      gpointer user_data)
  {
    PulseSrc *context = static_cast<PulseSrc *>(user_data);
    
    if (context->capture_device ((const char *)pulse_device_name))
      return TRUE;
    else
      return FALSE;
  }

  bool 
  PulseSrc::capture_device (const char *pulse_device_name)
  {
    make_elements ();

    if (g_strcmp0 (pulse_device_name, "NONE") != 0)
      if (capture_devices_.find (pulse_device_name) != capture_devices_.end ())	
     	g_object_set (G_OBJECT (pulsesrc_), "device", pulse_device_name, NULL);
      else
     	{
     	  g_warning ("PulseSrc: device %s has not been detected by pulse audio, cannot use", pulse_device_name);
     	  return false;
     	}
    
    set_raw_audio_element (pulsesrc_bin_);
   
    return true;
  }

}//end of PulseSrc class
  
  
  
