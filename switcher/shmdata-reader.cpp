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

#include "switcher/shmdata-reader.h"

namespace switcher
{

  bool ShmdataReader::async_handler_installed_ = false;

  GstBusSyncReply 
  ShmdataReader::bus_async_handler (GstBus * bus,
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
  
  ShmdataReader::~ShmdataReader()
  {
      shmdata_base_reader_close (reader_);

      std::vector<GstElement *>::iterator element;
      for (element = elements_to_remove_.begin(); element != elements_to_remove_.end (); element ++)
	{
	  GstIterator *pad_iter;
	  pad_iter = gst_element_iterate_pads (*element);
	  gst_iterator_foreach (pad_iter, (GFunc) unlink_pad, *element);
	  gst_iterator_free (pad_iter);
	  gst_element_set_state (*element, GST_STATE_NULL);
	  gst_bin_remove (GST_BIN (gst_element_get_parent (*element)), *element);
	}
  
      g_print ("ShmdataReader: %s deleted \n", name_.c_str());
  }

  ShmdataReader::ShmdataReader()
  {
    reader_ = shmdata_base_reader_new ();
    bin_ = NULL;
    name_ = "";
    sink_element_ = NULL;
    connection_hook_ = NULL;
   }
  
  void
  ShmdataReader::unlink_pad (GstPad * pad)
  {
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
    return name_;
  }

  void 
  ShmdataReader::set_path (const char *absolute_path)
  {
    name_ = absolute_path;
  }

  void 
  ShmdataReader::set_bin (GstElement *bin)
  {
    bin_ = bin;
  }

  void 
  ShmdataReader::set_sink_element (GstElement *sink_element)
  {   
    sink_element_ = sink_element;
  }

  void 
  ShmdataReader::start ()
  {
    //g_print ("shmdata-reader START \n");

    shmdata_base_reader_close (reader_);
    reader_ = shmdata_base_reader_new ();
    
    if (name_ == "" ||  bin_ == NULL)
      {
	g_printerr ("cannot start the shmdata reader: name or bin or sink element has not bin set\n");
	return;
      }
    
    if (!async_handler_installed_)
      {
	async_handler_installed_ = true;
	//looking for the bus, searching the top level pipeline
	GstElement *pipe = bin_;
	
	while (pipe != NULL && !GST_IS_PIPELINE (pipe))
	  pipe = GST_ELEMENT_PARENT (pipe);
	
	if( GST_IS_PIPELINE (pipe))
	  {
	    GstBus *bus = gst_pipeline_get_bus (GST_PIPELINE (pipe));  
	    gst_bus_set_sync_handler (bus, ShmdataReader::bus_async_handler, NULL);  
	    gst_object_unref (bus);  
	  }
	else
	  {
	    g_warning ("no top level pipeline found when starting, cannot install async_handler");
	    return;
	  }
      }
    
    shmdata_base_reader_set_callback (reader_, ShmdataReader::on_first_data, this);
    shmdata_base_reader_install_sync_handler (reader_, FALSE);
    shmdata_base_reader_set_bin (reader_, bin_);
    shmdata_base_reader_start (reader_, name_.c_str());

  }

  void 
  ShmdataReader::stop ()
  {
      shmdata_base_reader_close (reader_);
  } 
 
  void 
  ShmdataReader::add_element_to_remove (GstElement *element)
  {
    elements_to_remove_.push_back (element);
  }
  
  void 
  ShmdataReader::set_on_first_data_hook (on_first_data_hook cb, void *user_data)
  {
    //g_print ("ShmdataReader::set_on_first_data_hook\n");
    connection_hook_ = cb;
    hook_user_data_ = user_data;
  }
 
  void 
  ShmdataReader::on_first_data (shmdata_base_reader_t *context, void *user_data)
  {
      ShmdataReader *reader = static_cast<ShmdataReader *>(user_data);
  
      if (reader->connection_hook_ != NULL) //user want to create the sink_element_ 
	reader->connection_hook_ (reader, reader->hook_user_data_);
      else
	{	
	  
	  if (!GST_IS_ELEMENT (GST_ELEMENT_PARENT (reader->sink_element_)))
	    {
	      gst_bin_add (GST_BIN (reader->bin_), reader->sink_element_);
	      
	      gst_element_sync_state_with_parent (reader->sink_element_);
	      gst_element_sync_state_with_parent (reader->bin_);
	    }
	  else 
	    g_printerr ("ShmdataReader::on_first_data: sink element has not parent\n");
	}
      
      shmdata_base_reader_set_sink (context, reader->sink_element_);
      
  }

}
