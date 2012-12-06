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

#include "switcher/uridecodebin.h"
#include "switcher/gst-utils.h"
#include <glib/gprintf.h>
#include <memory>

namespace switcher
{
  const QuiddityDocumentation Uridecodebin::doc_ ("uri decoding", "uridecodebin", 
						  "decode an URI of live stream(s) to shmdata(s)");
  
  bool
  Uridecodebin::init() 
  { 
    media_counter_ = 0;
    uridecodebin_ = gst_element_factory_make ("uridecodebin",NULL);   
    //set the name before registering properties
    set_name (gst_element_get_name (uridecodebin_));
    add_element_to_cleaner (uridecodebin_);
    
    g_signal_connect (G_OBJECT (uridecodebin_), 
		      "pad-added", 
		      (GCallback) Uridecodebin::uridecodebin_pad_added_cb,
		      (gpointer) this);
    g_signal_connect (G_OBJECT (uridecodebin_),  
		      "no-more-pads",  
		      (GCallback) Uridecodebin::no_more_pads_cb ,  
		      (gpointer) this);      
    g_signal_connect (G_OBJECT (uridecodebin_),  
		      "source-setup",  
		      (GCallback) Uridecodebin::source_setup_cb ,  
		      (gpointer) this);      


    // g_signal_connect (G_OBJECT (uridecodebin_),  
    // 		      "pad-removed",  
    // 		      (GCallback) Uridecodebin::pad_removed_cb ,  
    // 		      (gpointer) this);      
    // g_signal_connect (G_OBJECT (uridecodebin_),  
    // 		      "unknown-type",  
    // 		      (GCallback) Uridecodebin::unknown_type_cb ,  
    // 		      (gpointer) this);      
    // g_signal_connect (G_OBJECT (uridecodebin_),  
    // 		      "autoplug-continue",  
    // 		      (GCallback) Uridecodebin::autoplug_continue_cb ,  
    // 		      (gpointer) this);      
    // g_signal_connect (G_OBJECT (uridecodebin_),  
    // 		      "autoplug-factory",  
    // 		      (GCallback) Uridecodebin::autoplug_factory_cb ,  
    // 		      (gpointer) this);      
    // g_signal_connect (G_OBJECT (uridecodebin_),  
    // 		      "autoplug-sort",  
    // 		      (GCallback) Uridecodebin::autoplug_sort_cb ,  
    // 		      (gpointer) this);      
    // g_signal_connect (G_OBJECT (uridecodebin_),  
    // 		      "autoplug-select",  
    // 		      (GCallback) Uridecodebin::autoplug_select_cb ,  
    // 		      (gpointer) this);      
    // g_signal_connect (G_OBJECT (uridecodebin_),  
    // 		      "drained",  
    // 		      (GCallback) Uridecodebin::drained_cb ,  
    // 		      (gpointer) this);      

    // g_signal_connect (G_OBJECT (uridecodebin_),  
    //  		    "drained",  
    //  		    (GCallback) uridecodebin_drained_cb ,  
    //  		    (gpointer) this);      
    g_object_set (G_OBJECT (uridecodebin_),  
    // 		  //"ring-buffer-max-size",(guint64)200000000, 
    // 		  //"download",TRUE, 
    // 		  //"use-buffering",TRUE, 
    // 		  "expose-all-streams", TRUE,
    		  "async-handling",FALSE, 
    // 		  //"buffer-duration",9223372036854775807, 
     NULL); 

   
    //registering add_data_stream
    register_method("to_shmdata",
		    (void *)&to_shmdata_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ("to_shmdata", 
			    "decode streams from an uri and write them to shmdatas", 
			    Method::make_arg_description ("uri", 
							  "the uri to decode",
							  NULL));
    return true;
  }
  
  QuiddityDocumentation 
  Uridecodebin::get_documentation ()
  {
    return doc_;
  }

  void 
  Uridecodebin::no_more_pads_cb (GstElement* object, gpointer user_data)   
  {   
    //g_print ("---- no more pad\n");
    // Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
  }


  void 
  Uridecodebin::uridecodebin_pad_added_cb (GstElement* object, GstPad* pad, gpointer user_data)   
  {   
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
    
    const gchar *padname= gst_structure_get_name (gst_caps_get_structure(gst_pad_get_caps (pad),0));
    g_debug ("uridecodebin new pad name is %s\n",padname);
    
    GstElement *identity = gst_element_factory_make ("identity",NULL);
    g_object_set (identity, "sync", TRUE, NULL);

    gst_bin_add (GST_BIN (context->bin_), identity);
    GstUtils::link_static_to_request (pad, identity);
    gst_element_sync_state_with_parent (identity);
    //gst_element_set_state (identity, GST_STATE_PLAYING);

    //giving a name to the stream
    gchar **padname_splitted = g_strsplit_set (padname, "/",-1);
    gchar media_name[256];
    g_sprintf (media_name,"%s_%d",padname_splitted[0],context->media_counter_);
    context->media_counter_++;
    g_strfreev(padname_splitted);

    //creating a shmdata
    ShmdataWriter::ptr connector;
    connector.reset (new ShmdataWriter ());
    std::string connector_name = context->make_file_name (media_name);
    connector->set_path (connector_name.c_str());
    GstCaps *caps = gst_pad_get_caps_reffed (pad);

    connector->plug (context->bin_, identity, caps);

    if (G_IS_OBJECT (caps))
      gst_object_unref (caps);
    context->shmdata_writers_.insert (connector_name, connector);

    g_message ("%s created a new shmdata writer (%s)", 
	       context->get_nick_name ().c_str(), 
	       connector_name.c_str ());
  }   

  void 
  Uridecodebin::source_setup_cb (GstElement *uridecodebin, GstElement *source, gpointer user_data)
  {
    //Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
    g_print ("source %s %s\n",  GST_ELEMENT_NAME(source), G_OBJECT_CLASS_NAME (G_OBJECT_GET_CLASS (source)));
  }

  gboolean
  Uridecodebin::to_shmdata_wrapped (gpointer uri, 
				    gpointer user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
  
    if (context->to_shmdata ((char *)uri))
      return TRUE;
    else
      return FALSE;
  }

  bool
  Uridecodebin::to_shmdata (std::string uri)
  {
    g_debug ("to_shmdata set uri %s", uri.c_str ());
    g_object_set (G_OBJECT (uridecodebin_), "uri", uri.c_str (), NULL); 

    gst_bin_add (GST_BIN (bin_), uridecodebin_);
    gst_element_sync_state_with_parent (uridecodebin_);
    //gst_element_set_state (uridecodebin_, GST_STATE_PAUSED);

    g_debug ("to_shmdata end");
    return true;
  }


}
