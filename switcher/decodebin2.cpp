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

#include "decodebin2.h"
#include "gst-element-cleaner.h"
#include "gst-utils.h"
#include <glib/gprintf.h>

namespace switcher
{
  QuiddityDocumentation Decodebin2::doc_ ("automatic decoding", "decodebin2", 
					     "connect to a shmdata, decode it and write decoded frames to shmdata(s)");
  
  bool
  Decodebin2::init() 
  { 
    if (!GstUtils::make_element ("decodebin2",&decodebin2_))
      return false;
    //set the name before registering properties
    set_name (gst_element_get_name (decodebin2_));
    add_element_to_cleaner (decodebin2_);
    set_sink_element (decodebin2_);
    set_on_first_data_hook (Decodebin2::make_decodebin2_active,this);
 
    g_signal_connect (G_OBJECT (decodebin2_), 
		      "pad-added", 
		      (GCallback) Decodebin2::pad_added_cb,
		      (gpointer) this);
   g_signal_connect (G_OBJECT (decodebin2_), 
		      "no-more-pads", 
		      (GCallback) Decodebin2::no_more_pads_cb,
		      (gpointer) this);
  
   return true;
  }
  
  QuiddityDocumentation 
  Decodebin2::get_documentation ()
  {
    return doc_;
  }

  void
  Decodebin2::make_decodebin2_active (ShmdataReader *caller, void *decodebin2_instance)
  {
    Decodebin2 *context = static_cast<Decodebin2 *>(decodebin2_instance);
    caller->set_sink_element (context->decodebin2_);
    gst_bin_add (GST_BIN (context->bin_), context->decodebin2_);
    GstUtils::sync_state_with_parent (context->decodebin2_);
  }
    
  void 
  Decodebin2::no_more_pads_cb (GstElement* object, gpointer user_data)   
  {   
    //g_print ("no more pad");
    //Decodebin2 *context = static_cast<Decodebin2 *>(user_data);
  }


  void 
  Decodebin2::pad_added_cb (GstElement* object, GstPad* pad, gpointer user_data)   
  {   
    Decodebin2 *context = static_cast<Decodebin2 *>(user_data);
    
    const gchar *padname= gst_structure_get_name (gst_caps_get_structure(gst_pad_get_caps (pad),0));
    g_debug ("decodebin2 new pad name is %s\n",padname);
    
    // GstElement *identity = gst_element_factory_make ("identity",NULL);
    // g_object_set (identity, "sync", TRUE, NULL);

    // gst_bin_add (GST_BIN (context->bin_), identity);
    // GstUtils::link_static_to_request (pad, identity);
    // GstUtils::sync_state_with_parent (identity);

    //preparing pad name
    gchar **padname_splitted = g_strsplit_set (padname, "/",-1);
    int count = 0;
    if (context->media_counters_.contains (std::string (padname_splitted[0])))
       {
	 count = context->media_counters_.lookup (std::string (padname_splitted[0]));
	 count = count+1;
       }
    context->media_counters_.replace (std::string (padname_splitted[0]), count);

    gchar media_name[256];
    g_sprintf (media_name,"%s_%d",padname_splitted[0],count);
    g_debug ("decodebin2: new media %s %d\n",media_name, count );
    g_strfreev(padname_splitted);
    
    //creating a shmdata
    ShmdataWriter::ptr connector;
    connector.reset (new ShmdataWriter ());
    std::string connector_name = context->make_file_name (media_name);
    connector->set_path (connector_name.c_str());
    connector->plug (context->bin_, pad);
    context->register_shmdata_writer (connector);
    g_message ("%s created a new shmdata writer (%s)", 
     	       context->get_nick_name ().c_str(), 
     	       connector_name.c_str ());
 
  }   

}
