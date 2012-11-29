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

#include "switcher/shmdata-writer.h"

namespace switcher
{

  ShmdataWriter::ShmdataWriter() :
    writer_ (shmdata_base_writer_init ())
  {
  }

  ShmdataWriter::~ShmdataWriter()
  {
    g_debug ("ShmdataWriter: deleting %s", path_.c_str());

    shmdata_base_writer_close (writer_);

    //cleaning tee_, queue_, fakesink_
    if (GST_IS_ELEMENT (tee_))
      {
	gst_element_set_state (tee_, GST_STATE_NULL);
	GstPad *sinkpad = gst_element_get_static_pad (tee_,"sink");
	GstPad *peer;
	if ((peer = gst_pad_get_peer (sinkpad))) 
	  {
	    gst_pad_unlink (peer, sinkpad);
	    GstPadTemplate *pad_templ = gst_pad_get_pad_template (peer);//TODO check if must be unrefed for GST 1
	    if (GST_PAD_TEMPLATE_PRESENCE (pad_templ) == GST_PAD_REQUEST)
	      gst_element_release_request_pad (gst_pad_get_parent_element(peer), peer);
	    gst_object_unref (peer);
	  }
	gst_object_unref (sinkpad);
	gst_bin_remove (GST_BIN (gst_element_get_parent (tee_)), tee_);
      }

    if (GST_IS_ELEMENT (queue_))
      {
	gst_element_set_state (queue_, GST_STATE_NULL);
	gst_bin_remove (GST_BIN (gst_element_get_parent (queue_)), queue_);
      }

    if (GST_IS_ELEMENT (fakesink_))
      {
	gst_element_set_state (fakesink_, GST_STATE_NULL);
	gst_bin_remove (GST_BIN (gst_element_get_parent (fakesink_)), fakesink_);
      }
    g_debug ("ShmdataWriter: %s deleted", path_.c_str());
  }
  
  //WARNING if the file exist it will be deleted
  bool 
  ShmdataWriter::set_path (std::string name)
  {
    GFile *shmfile = g_file_new_for_commandline_arg (name.c_str());
    if( g_file_query_exists (shmfile, NULL))
      {    
	//thrash it
	g_debug ("ShmdataWriter::set_path warning: file %s exists and will be deleted.",name.c_str());
	if (! g_file_delete (shmfile, NULL, NULL)) 
	  {
	    g_error ("ShmdataWriter::set_path error: file %s is already existing and cannot be trashed.",name.c_str());
	    return false;
	  }
      }
    
    //setting the writer
    shmdata_base_writer_set_path (writer_,name.c_str());
    path_ = name;
    return true;
  }
  
  void 
  ShmdataWriter::plug (GstElement *bin, GstElement *source_element, GstCaps *caps)
  {
     tee_ = gst_element_factory_make ("tee", NULL);
     queue_ = gst_element_factory_make ("queue", NULL); 
     fakesink_ = gst_element_factory_make ("fakesink", NULL); 
     g_object_set (G_OBJECT(fakesink_),"sync",FALSE,NULL);
    
     gst_bin_add_many (GST_BIN (bin), tee_, queue_, fakesink_, NULL);

     shmdata_base_writer_plug (writer_, bin, tee_);

     gst_element_link_filtered (source_element,
				tee_, caps);
     gst_element_link_many (tee_, queue_, fakesink_,NULL);
  
     gst_element_sync_state_with_parent (tee_);
     gst_element_sync_state_with_parent (queue_);
     gst_element_sync_state_with_parent (fakesink_);

     g_debug ("shmdata writer plugged (%s)",path_.c_str());
  }

  void 
  ShmdataWriter::plug (GstElement *bin, GstPad *source_pad)
  {
     tee_ = gst_element_factory_make ("tee", NULL);
     queue_ = gst_element_factory_make ("queue", NULL); 
     fakesink_ = gst_element_factory_make ("fakesink", NULL); 
     g_object_set (G_OBJECT(fakesink_),"sync",FALSE,NULL);
    
     gst_bin_add_many (GST_BIN (bin), tee_, queue_, fakesink_, NULL);

     shmdata_base_writer_plug (writer_, bin, tee_);

     GstPad *sinkpad = gst_element_get_static_pad (tee_, "sink");
     if (gst_pad_link (source_pad, sinkpad) != GST_PAD_LINK_OK)
       g_error ("ShmdataWriter: failed to link with tee");
     gst_object_unref (sinkpad);
     //gst_element_link_filtered (source_element,
     //				tee_, caps);
     gst_element_link_many (tee_, queue_, fakesink_,NULL);
     gst_element_sync_state_with_parent (tee_);
     gst_element_sync_state_with_parent (queue_);
     gst_element_sync_state_with_parent (fakesink_);
  }

}
