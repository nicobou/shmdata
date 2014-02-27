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
    gst_launch_pipeline_ (g_strdup ("videotestsrc is-live=true"))
  {}

  GstVideoParseToBinSrc::~GstVideoParseToBinSrc ()
  {
    g_free (gst_launch_pipeline_);
    if (NULL != gst_video_parse_to_bin_src_)
      GstUtils::clean_element (gst_video_parse_to_bin_src_);
  }

  bool 
  GstVideoParseToBinSrc::init_segment ()
  {
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
  GstVideoParseToBinSrc::make_video_source (GstElement **new_element)
  {
    g_debug ("trying to make video source from pipeline (%s)", gst_launch_pipeline_);
    if (NULL != gst_video_parse_to_bin_src_)
      GstUtils::clean_element (gst_video_parse_to_bin_src_);
    GError *error = NULL;
    gst_video_parse_to_bin_src_ = gst_parse_bin_from_description (gst_launch_pipeline_,
								  TRUE,
								  &error);
    g_object_set (G_OBJECT (gst_video_parse_to_bin_src_), "async-handling", TRUE, NULL);
    if (error != NULL)
      {
	g_debug ("%s",error->message);
	g_error_free (error);
	gst_video_parse_to_bin_src_ = NULL;
	return false;
      }
    GstPad *src_pad = gst_element_get_static_pad (gst_video_parse_to_bin_src_,"src");
    GstCaps *caps = gst_pad_get_caps (src_pad);
    gchar *string_caps = gst_caps_to_string (caps);
    if (!g_str_has_prefix (string_caps,"video/") && !g_str_has_prefix (string_caps,"ANY"))
      {
	g_debug ("description does not provide video (caps is %s)",string_caps);
	gst_caps_unref (caps);
	g_free (string_caps);
	gst_object_unref (src_pad);
	gst_video_parse_to_bin_src_ = NULL;
	return false;
      }
    g_free (string_caps);
    gst_caps_unref (caps);
    gst_object_unref (src_pad);
    *new_element = gst_video_parse_to_bin_src_;
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
  
  const gchar *
  GstVideoParseToBinSrc::get_gst_launch_pipeline (void *user_data)
  {
    GstVideoParseToBinSrc *context = static_cast <GstVideoParseToBinSrc *> (user_data);
    return context->gst_launch_pipeline_;
  }

  bool 
  GstVideoParseToBinSrc::on_start ()
  {
    //disable_property ("gst-pipeline");
    return true;
  }
  
  bool 
  GstVideoParseToBinSrc::on_stop ()
  {
    reset_bin ();
    //enable_property ("gst-pipeline");
    return true;
  }

}
