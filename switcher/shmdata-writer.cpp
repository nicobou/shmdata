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
  {}
  
  bool 
  ShmdataWriter::set_name (std::string name)
  {
    g_printerr ("ShmdataWriter::set_name (const char *name) IS NOT IMPLEMENTED");
    return false;
  }
  
  bool 
  ShmdataWriter::set_absolute_name (std::string name)
  {

    GFile *shmfile = g_file_new_for_commandline_arg (name.c_str());
    if( g_file_query_exists (shmfile, NULL))
      {    
	//thrash it
	g_printerr ("ShmdataWriter::set_absolute_name warning: file %s exists and will be trashed.\n",name.c_str());
	if (! g_file_trash (shmfile, NULL, NULL)) 
	  {
	    g_printerr("ShmdataWriter::set_absolute_name error: file %s is already existing and cannot be trashed.",name.c_str());
	    return false;
	  }
      }
    
    //setting the writer
    shmdata_base_writer_set_path (writer_,name.c_str());
    name_ = name;
    return true;
  }
  
  void 
  ShmdataWriter::plug (GstElement *bin, GstElement *source_element)
  {
    g_print ("coucou\n");
    GstElement *vts = gst_element_factory_make ("videotestsrc",NULL);
    g_object_set (G_OBJECT (vts), "is-live",TRUE,NULL);
    GstElement *tee = gst_element_factory_make ("tee",NULL);
    GstElement *queue = gst_element_factory_make ("queue", NULL); 
    GstElement *fakesink = gst_element_factory_make ("xvimagesink", NULL);
    gst_bin_add_many (GST_BIN (bin), vts, tee, queue, fakesink, NULL);
    gst_element_link_many (vts, tee, NULL);
    gst_element_link_many (tee, queue, fakesink, NULL);
    // gst_element_sync_state_with_parent (vts);
    // gst_element_sync_state_with_parent (tee);
    // gst_element_sync_state_with_parent (queue);
    // gst_element_sync_state_with_parent (fakesink);
    
    shmdata_base_writer_plug (writer_, bin, tee);
  }
  
}
