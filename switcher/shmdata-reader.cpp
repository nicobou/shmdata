/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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

#include "shmdata-reader.h"
#include "gst-utils.h"

namespace switcher
{

  ShmdataReader::ShmdataReader()
  {
    json_description_.reset (new JSONBuilder ());
    reader_ = shmdata_base_reader_new ();
    bin_ = NULL;
    path_ = "";
    sink_element_ = NULL;
    funnel_ = NULL;
    connection_hook_ = NULL;
   }

  ShmdataReader::~ShmdataReader()
  {
      g_debug ("ShmdataReader: deleting %s", path_.c_str());
      shmdata_base_reader_close (reader_);
      g_debug ("ShmdataReader: %s deleted ", path_.c_str());
  }

  void
  ShmdataReader::unlink_pad (GstPad * pad)
  {
    g_debug ("ShmdataReader::unlink_pad SHOULD NOT BE CALLED, ");
    GstPad *peer;
    if ((peer = gst_pad_get_peer (pad))) {
      if (gst_pad_get_direction (pad) == GST_PAD_SRC)
	gst_pad_unlink (pad, peer);
      else
	gst_pad_unlink (peer, pad);
      //checking if the pad has been requested and releasing it needed 
      GstPadTemplate *pad_templ = gst_pad_get_pad_template (peer);//check if this must be unrefed for GST 1
      if (GST_PAD_TEMPLATE_PRESENCE (pad_templ) == GST_PAD_REQUEST)
	gst_element_release_request_pad (gst_pad_get_parent_element(peer), peer);
      gst_object_unref (peer);
    }
  }

  std::string 
  ShmdataReader::get_path ()
  {
    return path_;
  }

  void 
  ShmdataReader::set_path (const char *absolute_path)
  {
    path_ = absolute_path;
    make_json_description ();
  }

  void 
  ShmdataReader::set_bin (GstElement *bin)
  {
    bin_ = bin;
  }

  void 
  ShmdataReader::set_g_main_context (GMainContext *context)
  {
    g_main_context_ = context;
  }


  void 
  ShmdataReader::set_sink_element (GstElement *sink_element)
  {   
    sink_element_ = sink_element;
  }

  void 
  ShmdataReader::start ()
  {
    g_debug ("shmdata-reader::start");
    shmdata_base_reader_close (reader_);
    GstUtils::clean_element (funnel_);
    reader_ = shmdata_base_reader_new ();
    shmdata_base_reader_set_g_main_context (reader_, g_main_context_);
    shmdata_base_reader_set_on_have_type_callback (reader_, ShmdataReader::on_have_type, this);
    if (path_ == "" ||  bin_ == NULL)
      {
	g_warning ("cannot start the shmdata reader: name or bin or sink element has not bin set");
	return;
      }
    
    // //looking for the bus, searching the top level pipeline
    // GstElement *pipe = bin_;
    // while (pipe != NULL && !GST_IS_PIPELINE (pipe))
    //   pipe = GST_ELEMENT_PARENT (pipe);
    // if( GST_IS_PIPELINE (pipe))
    //   {
    // 	GstBus *bus = gst_pipeline_get_bus (GST_PIPELINE (pipe));  
    // 	//clear old handler and install a new one 
    // 	gst_bus_set_sync_handler (bus, NULL, NULL);
    // 	gst_bus_set_sync_handler (bus, ShmdataReader::bus_sync_handler, NULL);  
    // 	gst_object_unref (bus);  
    //   }
    // else
    //   {
    // 	g_warning ("no top level pipeline found when starting, cannot install sync_handler");
    // 	return;
    //   }

    shmdata_base_reader_set_callback (reader_, ShmdataReader::on_first_data, this);
    shmdata_base_reader_install_sync_handler (reader_, FALSE);
    shmdata_base_reader_set_bin (reader_, bin_);
    shmdata_base_reader_start (reader_, path_.c_str());
    g_debug ("shmdata-reader::start done");
  }

  void 
  ShmdataReader::on_have_type (shmdata_base_reader_t *, 
			       GstCaps *caps, 
			       void *user_data)
  {
    ShmdataReader *reader = static_cast<ShmdataReader *>(user_data);
    reader->caps_ = caps;
    g_debug ("shmdata new caps: \n%s",gst_caps_to_string (reader->caps_));
  }

  void 
  ShmdataReader::stop ()
  {
    g_debug ("ShmdataReader::stop");
    shmdata_base_reader_close (reader_);
    GstUtils::clean_element (funnel_);
  } 
 
  void 
  ShmdataReader::set_on_first_data_hook (on_first_data_hook cb, void *user_data)
  {
    g_debug ("ShmdataReader::set_on_first_data_hook");
    connection_hook_ = cb;
    hook_user_data_ = user_data;
  }
 
  void 
  ShmdataReader::on_first_data (shmdata_base_reader_t *context, void *user_data)
  {
      ShmdataReader *reader = static_cast<ShmdataReader *>(user_data);
      g_debug (" ShmdataReader::on_first_data");
      if (reader->connection_hook_ != NULL) //user want to create the sink_element_ 
	reader->connection_hook_ (reader, reader->hook_user_data_);
      if (!GST_IS_ELEMENT (GST_ELEMENT_PARENT (reader->sink_element_)))
	  gst_bin_add (GST_BIN (reader->bin_), reader->sink_element_);
      // else 
      // 	  g_debug ("ShmdataReader::on_first_data: (%s) sink element (%s) has a parent (%s) %d", 
      // 		      reader->get_path ().c_str(), 
      // 		      GST_ELEMENT_NAME (reader->sink_element_), 
      // 		      GST_ELEMENT_NAME(GST_ELEMENT_PARENT (reader->sink_element_)),
      // 		      GST_IS_ELEMENT(GST_ELEMENT_PARENT (reader->sink_element_)));
      GstUtils::make_element ("funnel", &reader->funnel_);
      gst_bin_add (GST_BIN (reader->bin_), reader->funnel_);
      gst_element_link (reader->funnel_, reader->sink_element_);
      GstUtils::sync_state_with_parent (reader->sink_element_);
      GstUtils::sync_state_with_parent (reader->funnel_);
      shmdata_base_reader_set_sink (context, reader->funnel_);
  }

  GstCaps *
  ShmdataReader::get_caps () 
  {
    return caps_; 
  }

  void
  ShmdataReader::make_json_description ()
  {
    json_description_->reset ();
    json_description_->begin_object ();
    json_description_->add_string_member ("path", path_.c_str ());
    json_description_->end_object ();
  }

  JSONBuilder::Node 
  ShmdataReader::get_json_root_node ()
  {
    return json_description_->get_root ();
  }

}
