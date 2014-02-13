/*
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

#include "shmdata-writer.h"
#include "gst-utils.h"

namespace switcher
{

  ShmdataWriter::ShmdataWriter() :
    path_ (),
    writer_ (shmdata_base_writer_init ()),
    bin_ (NULL),
    tee_ (NULL),
    queue_ (NULL),
    fakesink_ (NULL),
    json_description_ (new JSONBuilder())
  {}

  ShmdataWriter::~ShmdataWriter()
  {
    //g_debug ("ShmdataWriter: cleaning elements %s", path_.c_str());
    if (NULL != tee_)
      GstUtils::clean_element (tee_);
    if (NULL != queue_)
      GstUtils::clean_element (queue_);
    if (NULL != fakesink_)
      GstUtils::clean_element (fakesink_);
    shmdata_base_writer_close (writer_);
    if (!path_.empty ())
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
	    g_debug ("ShmdataWriter::set_path error: file %s is already existing and cannot be trashed.",name.c_str());
	    return false;
	  }
      }
    
    return set_path_without_deleting (name);
  }

  bool 
  ShmdataWriter::set_path_without_deleting (std::string name)
  {
    //setting the writer
    shmdata_base_writer_set_path (writer_,name.c_str());
    path_ = name;
    make_json_description ();
    return true;
  }

  std::string 
  ShmdataWriter::get_path ()
  {
    return path_;
  }
 
  void 
  ShmdataWriter::plug (GstElement *bin, 
		       GstElement *source_element, 
		       GstCaps *caps)
  {
    g_debug ("ShmdataWriter::plug (source element)");
    bin_ = bin;
    GstUtils::make_element ("tee", &tee_);
    GstUtils::make_element ("queue", &queue_);
    GstUtils::make_element ("fakesink", &fakesink_);
    g_object_set (G_OBJECT(fakesink_),"sync",FALSE,NULL);
    gst_bin_add_many (GST_BIN (bin), tee_, queue_, fakesink_, NULL);
    shmdata_base_writer_plug (writer_, 
			      bin, 
			      tee_);
    gst_element_link_filtered (source_element, 
			       tee_, 
			       caps);
    gst_element_link_many (tee_, 
			   queue_, 
			   fakesink_,
			   NULL);
    GstUtils::sync_state_with_parent (tee_);
    GstUtils::sync_state_with_parent (queue_);
    GstUtils::sync_state_with_parent (fakesink_);
    if (!path_.empty ())
      g_debug ("shmdata writer plugged (%s)", path_.c_str());
  }

  void 
  ShmdataWriter::plug (GstElement *bin, 
		       GstPad *source_pad)
  {
    bin_ = bin;
    GstUtils::make_element ("tee", &tee_);
    GstUtils::make_element ("queue", &queue_);
    GstUtils::make_element ("fakesink", &fakesink_);
    g_object_set (G_OBJECT(fakesink_), "sync", FALSE, NULL);
    g_object_set (G_OBJECT(fakesink_), "silent", TRUE, NULL);
    gst_bin_add_many (GST_BIN (bin), 
     		      tee_, 
     		      queue_, 
     		      fakesink_,
     		      NULL);
    shmdata_base_writer_plug (writer_, 
			      bin, 
			      tee_);
    GstPad *sinkpad = gst_element_get_static_pad (tee_, 
						  "sink");
    if (gst_pad_link (source_pad, sinkpad) != GST_PAD_LINK_OK)
      g_debug ("ShmdataWriter: failed to link with tee");
    gst_object_unref (sinkpad);
    gst_element_link_many (tee_, 
			   queue_, 
			   fakesink_,
			   NULL);
    GstUtils::sync_state_with_parent (tee_);
    GstUtils::sync_state_with_parent (queue_);
    GstUtils::sync_state_with_parent (fakesink_);
    if (!path_.empty ())
      g_debug ("shmdata writer pad plugged (%s)", path_.c_str());
  }
  
  void
  ShmdataWriter::make_json_description ()
  {
    json_description_->reset ();
    json_description_->begin_object ();
    json_description_->add_string_member ("path", path_.c_str ());
    json_description_->end_object ();
  }

  JSONBuilder::Node 
  ShmdataWriter::get_json_root_node ()
  {
    return json_description_->get_root ();
  }

}
