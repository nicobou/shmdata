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
#include "switcher/gst-utils.h"
#include <shmdata/base-reader.h>

namespace switcher
{

  const QuiddityDocumentation Runtime::doc_ ("runtime", "runtime",
					     "Complete pipeline container and scheduler");

  bool
  Runtime::init ()
  {
    pipeline_ = gst_pipeline_new (NULL);
    set_name (gst_element_get_name (pipeline_));
    GstBus *bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline_)); 
    gst_bus_add_watch (bus, bus_called, NULL);
    gst_bus_set_sync_handler (bus, bus_sync_handler, NULL);  
    gst_object_unref (bus); 
    
    gst_element_set_state (pipeline_, GST_STATE_PLAYING);
    return true;
  }
  
  Runtime::~Runtime ()
  {
    g_debug ("deleting runtime");
    gst_element_set_state (pipeline_, GST_STATE_NULL);
    gst_object_unref (GST_OBJECT (pipeline_));
    g_debug ("runtime deleted");
  }

  GstElement * 
  Runtime::get_pipeline ()
  {
    return pipeline_;
  }

  GstBusSyncReply 
  Runtime::bus_sync_handler (GstBus * bus,
			     GstMessage * msg, gpointer user_data) 
  {
    shmdata_base_reader_t *reader = (shmdata_base_reader_t *) g_object_get_data (G_OBJECT (msg->src), 
										 "shmdata_base_reader");
    if ( reader != NULL)
      {
	if ( shmdata_base_reader_process_error (reader, msg)) 
	  return GST_BUS_DROP; 
	else 
	  return GST_BUS_PASS; 
      }
    
    return GST_BUS_PASS; 
  }

  gboolean
  Runtime::bus_called (GstBus *bus,
		     GstMessage *msg,
		     gpointer data)
  {
    
    switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_EOS:
      g_debug ("bus_call End of stream, name: %s",
	       GST_MESSAGE_SRC_NAME(msg));
      break;
    case GST_MESSAGE_SEGMENT_DONE:
      g_debug ("bus_call segment done");
      break;
    case GST_MESSAGE_ERROR: 
      gchar *debug;
      GError *error;
      
      gst_message_parse_error (msg, &error, &debug);
      g_free (debug);

      
      
      // shmdata_base_reader_t *reader = (shmdata_base_reader_t *) g_object_get_data (G_OBJECT (msg->src), "shmdata_base_reader");
      // if ( reader != NULL)
      // 	shmdata_base_reader_process_error (reader, msg);
      
      g_error ("bus_call Error: %s from %s", error->message, GST_MESSAGE_SRC_NAME(msg));
      g_error_free (error);
      return FALSE;

      break;
    case GST_MESSAGE_STATE_CHANGED:
      // GstState old_state, new_state;
      // gst_message_parse_state_changed (msg, &old_state, &new_state, NULL);
      // g_debug ("Element %s changed state from %s to %s.",
      // 	 GST_OBJECT_NAME (msg->src),
      // 	 gst_element_state_get_name (old_state),
      // 	 gst_element_state_get_name (new_state));
      break;
    default:
      //g_debug ("message %s from %s",GST_MESSAGE_TYPE_NAME(msg),GST_MESSAGE_SRC_NAME(msg));
      break;
    }
    return TRUE;
  }

  QuiddityDocumentation 
  Runtime::get_documentation ()
  {
    return doc_;
  }
  
  
}

