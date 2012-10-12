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
  
  bool Runtime::initialized_ = false;

  Runtime::Runtime ()
  {
    make_runtime ();
  }

  Runtime::Runtime (QuiddityLifeManager::ptr life_manager)
  {
    life_manager_ = life_manager;
    make_runtime ();
  }
  
  void
  Runtime::make_runtime ()
  {
    if (!initialized_)
      {
	gst_init (NULL,NULL);
	mainloop_ = g_main_loop_new (NULL, FALSE);
	thread_ = g_thread_new ("SwitcherMainLoop", GThreadFunc(main_loop_thread), this);
	initialized_ = true;
      }
    
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
    //FIXME count instances and do quit main loop
    //g_main_loop_quit (mainloop_);
    g_print ("runtime deleted\n");
  }

  gpointer
  Runtime::main_loop_thread (gpointer user_data)
  {
    Runtime *context = static_cast<Runtime*>(user_data);
    g_main_loop_run (context->mainloop_);
    return NULL;
  }

  GstElement * 
  Runtime::get_pipeline ()
  {
    return pipeline_;
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
      
      // shmdata_base_reader_t *reader = (shmdata_base_reader_t *) g_object_get_data (G_OBJECT (msg->src), "shmdata_base_reader");
      // if ( reader != NULL)
      // 	shmdata_base_reader_process_error (reader, msg);

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

  QuiddityDocumentation Runtime::doc_ ("runtime", "runtime",
					 "Complete pipeline container and scheduler");
  
  QuiddityDocumentation 
  Runtime::get_documentation ()
  {
    return doc_;
  }
  
  
}

