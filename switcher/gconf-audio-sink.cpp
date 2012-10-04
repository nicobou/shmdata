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

#include "switcher/gconf-audio-sink.h"
#include <gst/gst.h>

namespace switcher
{

  GconfAudioSink::GconfAudioSink ()
  {
    //FIXME use async queue in base quiddity manager in order to avoid that (constructor called in the right thread)
    data_cond_ = g_cond_new (); 

    data_mutex_ = g_mutex_new ();
    
    g_main_context_invoke (NULL, (GSourceFunc) GconfAudioSink::do_init, (gpointer) this);
    
    g_mutex_lock (data_mutex_);
    g_cond_wait (data_cond_, data_mutex_);
    g_mutex_unlock (data_mutex_);
  }


  gboolean 
  GconfAudioSink::do_init(gpointer user_data)
  {
    GconfAudioSink *context = static_cast<GconfAudioSink *>(user_data);

    g_mutex_lock (context->data_mutex_);

    context->audiobin_ = gst_element_factory_make ("bin",NULL);
    context->audioconvert_ = gst_element_factory_make ("audioconvert",NULL);
    context->resample_ = gst_element_factory_make ("audioresample",NULL);
    context->gconfaudiosink_ = gst_element_factory_make ("gconfaudiosink",NULL);
    g_object_set (G_OBJECT (context->gconfaudiosink_), "profile",2,NULL); //profile "music"

    // GstElement *testsrc = gst_element_factory_make ("audiotestsrc",NULL);
    // GstElement *testsink = gst_element_factory_make ("pulsesink",NULL);

    gst_bin_add_many (GST_BIN (context->audiobin_),
		      // testsrc,
		      // testsink,
		      context->audioconvert_,
		      context->resample_,
		      context->gconfaudiosink_,
		      NULL);
    
    // gst_element_link_many (testsrc,
    // 			   testsink,
    // 			   NULL);
    

    gst_element_link_many (context->audioconvert_,
			   context->resample_,
			   context->gconfaudiosink_,
			   NULL);
    
    GstPad *sink_pad = gst_element_get_static_pad (context->audioconvert_, "sink");
    GstPad *ghost_sinkpad = gst_ghost_pad_new (NULL, sink_pad);
    gst_pad_set_active(ghost_sinkpad,TRUE);
    gst_element_add_pad (context->audiobin_, ghost_sinkpad); 

    //FIXME unref sinkpad


    //set the name
    context->name_ = gst_element_get_name (context->gconfaudiosink_);
    context->set_sink_element (context->audiobin_);

    g_cond_signal (context->data_cond_);
    g_mutex_unlock (context->data_mutex_);


    // //listing all the properties
    // guint numproperty;
    // GParamSpec **property = g_object_class_list_properties (G_OBJECT_GET_CLASS(context->gconfaudiosink_), &numproperty);
    // for (guint i = 0; i < numproperty; i++) {
    //   //register_property (G_OBJECT (context->gconfaudiosink_),property[i]);
    //   Property *prop = new Property (G_OBJECT (context->gconfaudiosink_),property[i]);
    //   prop->print();
    // }


    g_print ("GconfAudioSink: WARNING, set sync property of the sinks to false\n");
    // //registering sync_sink
    // std::vector<GType> connect_arg_types;
    // connect_arg_types.push_back (G_TYPE_BOOLEAN);
    // context->register_method("sync_sinks",(void *)&GconfAudioSink::sync_sinks_wrapped, connect_arg_types,(gpointer)context);
    
    // std::vector<std::pair<std::string,std::string> > arg_desc;
    // std::pair<std::string,std::string> sync;
    // sync.first = "sync";
    // sync.second = "true for synchronizing, false otherwise";
    // arg_desc.push_back (sync); 
    // if (!context->set_method_description ("sync_sinks", "synchronize sink(s) of the element on the pipeline clock", arg_desc))
    //   g_printerr ("gconfaudiosink: cannot set method description for \"sync_sinks\"\n");

    return FALSE; //the source should be removed from the main loop
  }




   gboolean
   GconfAudioSink::sync_sinks_wrapped (gpointer do_sync, gpointer user_data)
  {

    //FIXME THIS DOES NOT WORK

    //std::string connector = static_cast<std::string>(connector_name);
    GconfAudioSink *context = static_cast<GconfAudioSink*>(user_data);

    //g_print ("GconfAudioSink: %s\n",gst_element_get_name (context->gconfaudiosink_));

    // if (g_object_class_find_property (G_OBJECT_GET_CLASS (G_OBJECT(context->gconfaudiosink_)), "sync"))
    //   {
    // 	g_print ("GconfAudioSink: sync_sinks gconfaudiosink\n");
    // 	g_object_set (G_OBJECT (context->gconfaudiosink_), "sync", FALSE, NULL);
    //   }

    GValue value = G_VALUE_INIT;
    g_value_init (&value, G_TYPE_BOOLEAN);
    g_value_set_boolean (&value, FALSE);
    
    gst_child_proxy_set_property (GST_OBJECT (context->gconfaudiosink_), "testsink::sync", &value);
    

    GstIterator *it = gst_bin_iterate_sinks (GST_BIN (context->gconfaudiosink_));
    
    
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
		  {
		    g_object_set (G_OBJECT (item), "sync", FALSE, NULL);
		    g_print ("GconfAudioSink: setting sink sync to false %s\n",gst_element_get_name (GST_OBJECT(item))); 
		  }
		else
		  g_printerr("Internal sink doesn't have sync property");
			
		
		g_print ("-----------%u\n",gst_child_proxy_get_children_count  ((GstChildProxy *)context->gconfaudiosink_));
		
		 
		// if (GST_IS_BIN (GST_OBJECT(item)))
		//   g_print ("item is bin\n");

		gst_object_unref (GST_OBJECT(item));
		break;
	      case GST_ITERATOR_RESYNC:
		g_print ("gst_iterator_resync\n");
		gst_iterator_resync (it);
		break;
	      case GST_ITERATOR_ERROR:
		g_printerr ("...wrong parameters were given...\n");
		done = TRUE;
		break;
	      case GST_ITERATOR_DONE:
		g_print ("GST_ITERATOR_DONE\n");
		done = TRUE;
		break;
	      }
	  }
	gst_iterator_free (it);
      }
    else
      {
	g_printerr ("GconfAudioSink is not a bin\n");
      }
    return TRUE;
  }

  QuiddityDocumentation GconfAudioSink::doc_ ("audio sink", "gconfaudiosink", 
						"Audio sink embedding the GConf-settings for audio output");
  
  QuiddityDocumentation 
  GconfAudioSink::get_documentation ()
  {
    return doc_;
  }
  
}
