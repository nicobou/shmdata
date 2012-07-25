/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * The Runtime class
 */

#include "switcher/runtime.h"

namespace switcher
{

  Runtime::Runtime (){

    gst_init (NULL,NULL);
    mainloop_ = g_main_loop_new (NULL, FALSE);
        
    pipeline_ = gst_pipeline_new (NULL);
    name_ = gst_element_get_name (pipeline_);
    bus_ = gst_pipeline_get_bus (GST_PIPELINE (pipeline_)); 
    gst_bus_add_watch (bus_, bus_called, NULL);
    gst_object_unref (bus_); 

    gst_element_set_state (pipeline_, GST_STATE_PLAYING);
  }

  Runtime::~Runtime ()
  {
    g_print ("deleting runtime\n");
    gst_element_set_state (pipeline_, GST_STATE_NULL);
    gst_object_unref (GST_OBJECT (pipeline_));
    g_main_loop_quit (mainloop_);
  }


  GstElement * 
  Runtime::get_pipeline ()
  {
    return pipeline_;
  }
  // void
  // Runtime::add_segment (Segment::ptr segment)
  // {
  //   g_print ("add segment %s\n",segment->get_name().c_str());
  //   gst_bin_add (GST_BIN (pipeline_),segment->get_bin());
  //   gst_element_sync_state_with_parent (segment->get_bin());
  // }


  void
  Runtime::run ()
  {
    g_main_loop_run (mainloop_);
  }

  gboolean
  Runtime::bus_called (GstBus *bus,
		     GstMessage *msg,
		     gpointer data)
  {
    
    switch (GST_MESSAGE_TYPE (msg)) {
      
    case GST_MESSAGE_EOS:
      g_print ("bus_call End of stream, name: %s\n",
	       GST_MESSAGE_SRC_NAME(msg));
      break;
    case GST_MESSAGE_SEGMENT_DONE:
      g_print ("bus_call segment done\n");
      break;
    case GST_MESSAGE_ERROR: {
      gchar *debug;
      GError *error;
    
      gst_message_parse_error (msg, &error, &debug);
      g_free (debug);
    
      g_printerr ("bus_call Error: %s\n", error->message);
      g_error_free (error);
    
      break;
    }
    default:
      //g_print ("unknown message type \n");
      break;
    }
    return TRUE;
  }
  
}

