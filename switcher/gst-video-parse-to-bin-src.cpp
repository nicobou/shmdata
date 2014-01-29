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

#include "gst-video-parse-to-bin-src.h"
#include "gst-utils.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GstVideoParseToBinSrc,
				       "GStreamer Video Pipeline",
				       "video source", 
				       "GStreamer (src) video pipeline description to a *single* shmdata",
				       "LGPL",
				       "gstvideosrc",
				       "Nicolas Bouillot");
  
  GstVideoParseToBinSrc::GstVideoParseToBinSrc () :
    gst_video_parse_to_bin_src_ (NULL),
    custom_props_ (new CustomPropertyHelper ()),
    gst_launch_pipeline_spec_ (NULL),
    gst_launch_pipeline_ (g_strdup (""))
  {}

  GstVideoParseToBinSrc::~GstVideoParseToBinSrc ()
  {
    g_free (gst_launch_pipeline_);
    if (GST_IS_ELEMENT (gst_video_parse_to_bin_src_))
      GstUtils::clean_element (gst_video_parse_to_bin_src_);
  }

  bool 
  GstVideoParseToBinSrc::init_segment ()
  {
    init_startable (this);
    gst_launch_pipeline_spec_ = 
      custom_props_->make_string_property ("gst-pipeline", 
					   "GStreamer Launch Source Pipeline",
					   "",
					   (GParamFlags) G_PARAM_READWRITE,
					   GstVideoParseToBinSrc::set_gst_launch_pipeline,
					   GstVideoParseToBinSrc::get_gst_launch_pipeline,
					   this);
    install_property_by_pspec (custom_props_->get_gobject (), 
				gst_launch_pipeline_spec_, 
				"gst-pipeline",
				"GStreamer Pipeline");

    return true;
  }
  
  bool
  GstVideoParseToBinSrc::to_shmdata ()
  {
    g_debug ("to_shmdata set GStreamer description %s", gst_launch_pipeline_);
    
    GError *error = NULL;
    gst_video_parse_to_bin_src_ = gst_parse_bin_from_description (gst_launch_pipeline_,
								  TRUE,
								  &error);
    if (error != NULL)
      {
	g_debug ("%s",error->message);
	g_error_free (error);
	return false;
      }
    
    GstPad *src_pad = gst_element_get_static_pad (gst_video_parse_to_bin_src_,"src");

    //g_debug ("pad current caps: %s", gst_caps_to_string (gst_pad_get_caps (src_pad)));
    GstCaps * caps = gst_pad_get_caps (src_pad);
    gchar *string_caps = gst_caps_to_string (caps);
    if (!g_str_has_prefix (string_caps,"video/") && !g_str_has_prefix (string_caps,"ANY"))
      {
	g_debug ("description does not provide video (caps is %s)",string_caps);
	g_free (string_caps);
	return false;
      }
    g_free (string_caps);
    
    //creating a connector for raw audio
    ShmdataWriter::ptr writer;
    writer.reset (new ShmdataWriter ());
    std::string writer_name = make_file_name ("video");
    writer->set_path (writer_name.c_str());

    gst_bin_add (GST_BIN (bin_), gst_video_parse_to_bin_src_);
    writer->plug (bin_, src_pad);

    //GstUtils::wait_state_changed (bin_);
    GstUtils::sync_state_with_parent (gst_video_parse_to_bin_src_);

    register_shmdata_writer (writer);
    
    gst_object_unref (src_pad);
    return true;
  }

  void 
  GstVideoParseToBinSrc::set_gst_launch_pipeline (const gchar *value, void *user_data)
  {
    GstVideoParseToBinSrc *context = static_cast <GstVideoParseToBinSrc *> (user_data);
    g_free (context->gst_launch_pipeline_);
    context->gst_launch_pipeline_ = g_strdup (value);
    context->custom_props_->notify_property_changed (context->gst_launch_pipeline_spec_);
   }
  
  gchar *
  GstVideoParseToBinSrc::get_gst_launch_pipeline (void *user_data)
  {
    GstVideoParseToBinSrc *context = static_cast <GstVideoParseToBinSrc *> (user_data);
    return context->gst_launch_pipeline_;
  }

  bool 
  GstVideoParseToBinSrc::clean ()
  {
    reset_bin ();
    return unregister_shmdata_writer (make_file_name ("video"));
  }
  
  bool 
  GstVideoParseToBinSrc::start ()
  {
    clean ();
    if (! to_shmdata ())
      return false;
    uninstall_property ("gst-pipeline");
    return true;
  }
  
  bool 
  GstVideoParseToBinSrc::stop ()
  {
    clean ();
    uninstall_property ("gst-pipeline");
    install_property_by_pspec (custom_props_->get_gobject (), 
				gst_launch_pipeline_spec_, 
				"gst-pipeline",
				"GStreamer Pipeline");
    return true;
  }

}
