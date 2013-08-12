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

#include "gst-parse-to-bin-src.h"
#include "gst-utils.h"

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GstParseToBinSrc,
				       "GStreamer Pipeline",
				       "source",
				       "GStreamer (src) pipeline description to a *single* shmdata",
				       "LGPL",
				       "gstsrc", 
				       "Nicolas Bouillot");
  
  GstParseToBinSrc::~GstParseToBinSrc ()
  {
    if (gst_parse_to_bin_src_ != NULL)
      GstUtils::clean_element (gst_parse_to_bin_src_);
  }

  bool 
  GstParseToBinSrc::init ()
  {
    //using parent bin name
    set_name (gst_element_get_name (bin_));
    gst_parse_to_bin_src_ = NULL;


    if (!publish_method ("GST To Shmdata (With Path)",
			 "to_shmdata_with_path", 
			 "make a bin from GStreamer description and make shmdata writer(s)", 
			 "success or fail",
			 Method::make_arg_description ("GStreamer Pipeline",
						       "gst_pipeline", 
						       "the GStreamer pipeline with no sink to instanciate",
						       "Shmdata Path",
						       "shmdata_path", 
						       "the path used for the shmdata",
						       NULL),
			 (Method::method_ptr) &to_shmdata_with_path_wrapped, 
			 G_TYPE_BOOLEAN,
			 Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, NULL),
			 true,
			 false,
			 this))
      return false;	
    
    if (!publish_method ("GST To Shmdata",
      			 "to_shmdata", 
      			 "make a bin from GStreamer description and make shmdata writer(s)", 
      			 "success or fail",
      			 Method::make_arg_description ("GStreamer Pipeline",
      						       "gst_pipeline", 
      						       "the GStreamer pipeline with no sink to instanciate",
      						       NULL),
      			 (Method::method_ptr) &to_shmdata_wrapped, 
      			 G_TYPE_BOOLEAN,
      			 Method::make_arg_type_description (G_TYPE_STRING, NULL),
			 true,
			 false,
      			 this))
      return false;
    
     return true;
  }
  


  gboolean
  GstParseToBinSrc::to_shmdata_wrapped (gpointer descr, 
					gpointer user_data)
  {
    GstParseToBinSrc *context = static_cast<GstParseToBinSrc *>(user_data);
  
    if (context->to_shmdata ((char *)descr))
      return TRUE;
    else
      return FALSE;
  }

  gboolean
  GstParseToBinSrc::to_shmdata_with_path_wrapped (gpointer descr, 
						  gpointer shmdata_path,
						  gpointer user_data)
  {
    GstParseToBinSrc *context = static_cast<GstParseToBinSrc *>(user_data);
    
    if (context->to_shmdata_with_path ((char *)descr, (char *)shmdata_path))
      return TRUE;
    else
      return FALSE;
  }

  bool
  GstParseToBinSrc::to_shmdata_with_path (std::string descr,
					  std::string shmdata_path)
  {

    GError *error = NULL;
    gst_parse_to_bin_src_ = gst_parse_bin_from_description (descr.c_str (),
							    TRUE,
							    &error);
    g_object_set (G_OBJECT (gst_parse_to_bin_src_), "async-handling",TRUE, NULL);

    GstUtils::wait_state_changed (bin_);

    if (error != NULL)
      {
	g_warning ("%s",error->message);
	g_error_free (error);
	return false;
      }
    
    GstPad *src_pad = gst_element_get_static_pad (gst_parse_to_bin_src_,"src");
    gst_bin_add (GST_BIN (bin_), gst_parse_to_bin_src_);

     //make a shmwriter
     ShmdataWriter::ptr writer;
     writer.reset (new ShmdataWriter ());
     writer->set_path (shmdata_path.c_str());
     writer->plug (bin_, src_pad);
     register_shmdata_writer (writer);
     gst_object_unref (src_pad);

     GstUtils::sync_state_with_parent (gst_parse_to_bin_src_);
     return true;
  }
  
  bool
  GstParseToBinSrc::to_shmdata (std::string descr)
  {
    g_debug ("to_shmdata set GStreamer description %s", descr.c_str ());
    std::string writer_name = make_file_name ("gstsrc"); //FIXME use caps name
    return to_shmdata_with_path (descr, writer_name);
  }

}
