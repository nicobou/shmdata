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

#include "gst-video-parse-to-bin-src.h"
#include "gst-utils.h"

namespace switcher
{

  QuiddityDocumentation GstVideoParseToBinSrc::doc_ ("source", "gstvideosrc",
						     "GStreamer (src) video pipeline description to a *single* shmdata");
  
  QuiddityDocumentation 
  GstVideoParseToBinSrc::get_documentation ()
  {
    return doc_;
  }
  
  

  bool 
  GstVideoParseToBinSrc::init ()
  {
    //using parent bin name
    set_name (gst_element_get_name (bin_));

    //registering add_data_stream
    register_method("to_shmdata",
		    (void *)&to_shmdata_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ("to_shmdata", 
			    "make a bin from GStreamer description and make shmdata writer(s)", 
			    Method::make_arg_description ("description", 
							  "the description to instanciate",
							  NULL));
  

    return true;
  }
  


  gboolean
  GstVideoParseToBinSrc::to_shmdata_wrapped (gpointer descr, 
					     gpointer user_data)
  {
    GstVideoParseToBinSrc *context = static_cast<GstVideoParseToBinSrc *>(user_data);
  
    if (context->to_shmdata ((char *)descr))
      return TRUE;
    else
      return FALSE;
  }

  bool
  GstVideoParseToBinSrc::to_shmdata (std::string descr)
  {
    g_debug ("to_shmdata set GStreamer description %s", descr.c_str ());
    
    GError *error = NULL;
    gst_video_parse_to_bin_src_ = gst_parse_bin_from_description (descr.c_str (),
								  TRUE,
								  &error);
    if (error != NULL)
      {
	g_warning ("%s",error->message);
	g_error_free (error);
	return false;
      }
    
    GstPad *src_pad = gst_element_get_static_pad (gst_video_parse_to_bin_src_,"src");

    //g_debug ("pad current caps: %s", gst_caps_to_string (gst_pad_get_caps (src_pad)));
    GstCaps * caps = gst_pad_get_caps (src_pad);
    gchar *string_caps = gst_caps_to_string (caps);
    if (!g_str_has_prefix (string_caps,"video/") && !g_str_has_prefix (string_caps,"ANY"))
      {
	g_warning ("description does not provide video (caps is %s)",string_caps);
	g_free (string_caps);
	return false;
      }
    g_free (string_caps);

    gst_bin_add (GST_BIN (bin_), gst_video_parse_to_bin_src_);
    GstUtils::wait_state_changed (bin_);
    GstUtils::sync_state_with_parent (gst_video_parse_to_bin_src_);
    
    //creating a connector for raw audio
    ShmdataWriter::ptr writer;
    writer.reset (new ShmdataWriter ());
    std::string writer_name = make_file_name ("video");
    writer->set_path (writer_name.c_str());
    writer->plug (bin_, src_pad);
    register_shmdata_writer (writer);
    
    gst_object_unref (src_pad);
    return true;
  }



}
