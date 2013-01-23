/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#include "switcher/gconf-video-sink.h"
#include <gst/gst.h>

namespace switcher
{

  bool
  GconfVideoSink::init ()
  {
    data_cond_ = g_cond_new (); 
    data_mutex_ = g_mutex_new ();
    g_main_context_invoke (NULL, (GSourceFunc) GconfVideoSink::do_init, (gpointer) this);
    g_mutex_lock (data_mutex_);
    g_cond_wait (data_cond_, data_mutex_);
    g_mutex_unlock (data_mutex_);
    return true;
  }
  
  gboolean 
  GconfVideoSink::do_init(gpointer user_data)
  {
    GconfVideoSink *context = static_cast<GconfVideoSink *>(user_data);

    g_mutex_lock (context->data_mutex_);

    context->gconfvideosink_ = gst_element_factory_make ("gconfvideosink",NULL);
    
    //set the name
    context->set_name (gst_element_get_name (context->gconfvideosink_));
    context->set_sink_element (context->gconfvideosink_);

    g_cond_signal (context->data_cond_);
    g_mutex_unlock (context->data_mutex_);

    g_debug ("GconfVideoSink: WARNING, set sync property of the sinks to false");
    // //registering sync_sink
    // std::vector<GType> connect_arg_types;
    // connect_arg_types.push_back (G_TYPE_BOOLEAN);
    // context->register_method("sync_sinks",(void *)&GconfVideoSink::sync_sinks_wrapped, connect_arg_types,(gpointer)context);
    
    // std::vector<std::pair<std::string,std::string> > arg_desc;
    // std::pair<std::string,std::string> sync;
    // sync.first = "sync";
    // sync.second = "true for synchronizing, false otherwise";
    // arg_desc.push_back (sync); 
    // if (!context->set_method_description ("sync_sinks", "synchronize sink(s) of the element on the pipeline clock", arg_desc))
    //   g_error ("gconfvideosink: cannot set method description for \"sync_sinks\"");

    return FALSE; //the source should be removed from the main loop
  }


   gboolean
   GconfVideoSink::sync_sinks_wrapped (gpointer do_sync, gpointer user_data)
  {

    //FIXME THIS DOES NOT WORK

    //std::string connector = static_cast<std::string>(connector_name);
    GconfVideoSink *context = static_cast<GconfVideoSink*>(user_data);

    if (g_object_class_find_property (G_OBJECT_GET_CLASS (G_OBJECT(context->gconfvideosink_)), "sync"))
      g_object_set (G_OBJECT (context->gconfvideosink_), "sync", FALSE, NULL);


    GstIterator *it = gst_bin_iterate_sinks (GST_BIN (context->gconfvideosink_));
    if (it !=NULL)
      {
	void *item;
	gboolean done = FALSE;
	while (!done) 
	  {
	    switch (gst_iterator_next (it, &item)) 
	      {
	      case GST_ITERATOR_OK:
		if (g_object_class_find_property (G_OBJECT_GET_CLASS (G_OBJECT(item)), "sync"))
		  g_object_set (G_OBJECT (item), "sync", FALSE, NULL);
		else
		  g_error("Internal sink doesn't have sync property");
		
		gst_object_unref (GST_OBJECT(item));
		break;
	      case GST_ITERATOR_RESYNC:
		gst_iterator_resync (it);
		break;
	      case GST_ITERATOR_ERROR:
		g_error ("...wrong parameters were given...");
		done = TRUE;
		break;
	      case GST_ITERATOR_DONE:
		done = TRUE;
		break;
	      }
	  }
	gst_iterator_free (it);
	
      }
    return TRUE;
  }

  QuiddityDocumentation GconfVideoSink::doc_ ("video sink", "gconfvideosink", 
						"Video sink embedding the GConf-settings for video output");
  
  QuiddityDocumentation 
  GconfVideoSink::get_documentation ()
  {
    return doc_;
  }
  

}
