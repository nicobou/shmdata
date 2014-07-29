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
  
  GstParseToBinSrc::GstParseToBinSrc () :
    gst_parse_to_bin_src_ (nullptr),
    custom_props_ (new CustomPropertyHelper ()),
    gst_launch_pipeline_spec_ (nullptr),
    gst_launch_pipeline_ (g_strdup (""))
  {}

  GstParseToBinSrc::~GstParseToBinSrc ()
  {
    g_free (gst_launch_pipeline_);
    if (gst_parse_to_bin_src_ != nullptr)
      GstUtils::clean_element (gst_parse_to_bin_src_);
  }

  bool 
  GstParseToBinSrc::init_gpipe ()
  {
    init_startable (this);
    gst_launch_pipeline_spec_ = 
      custom_props_->make_string_property ("gst-pipeline", 
					   "GStreamer Launch Source Pipeline",
					   "videotestsrc is-live=true",
					   (GParamFlags) G_PARAM_READWRITE,
					   GstParseToBinSrc::set_gst_launch_pipeline,
					   GstParseToBinSrc::get_gst_launch_pipeline,
					   this);
    install_property_by_pspec (custom_props_->get_gobject (), 
				gst_launch_pipeline_spec_, 
				"gst-pipeline",
				"GStreamer Live Source Pipeline");
    
     return true;
  }
  
  bool
  GstParseToBinSrc::to_shmdata ()
  {

    GError *error = nullptr;
    gst_parse_to_bin_src_ = gst_parse_bin_from_description (gst_launch_pipeline_,
							    TRUE,
							    &error);

    if (error != nullptr)
      {
	g_debug ("%s",error->message);
	g_error_free (error);
	gst_parse_to_bin_src_ = nullptr;
	return false;
      }

    g_object_set (G_OBJECT (gst_parse_to_bin_src_), "async-handling",TRUE, nullptr);
    //GstUtils::wait_state_changed (bin_);
    
    GstPad *src_pad = gst_element_get_static_pad (gst_parse_to_bin_src_,"src");
    gst_bin_add (GST_BIN (bin_), gst_parse_to_bin_src_);

     //make a shmwriter
     ShmdataWriter::ptr writer;
     writer.reset (new ShmdataWriter ());
     writer->set_path (make_file_name ("gstsrc").c_str ());//FIXME use caps name
     writer->plug (bin_, src_pad);
     register_shmdata (writer);
     gst_object_unref (src_pad);
     GstUtils::sync_state_with_parent (gst_parse_to_bin_src_);
     return true;
  }
  
  void 
  GstParseToBinSrc::set_gst_launch_pipeline (const gchar *value, void *user_data)
  {
    GstParseToBinSrc *context = static_cast <GstParseToBinSrc *> (user_data);
    g_free (context->gst_launch_pipeline_);
    context->gst_launch_pipeline_ = g_strdup (value);
    context->custom_props_->notify_property_changed (context->gst_launch_pipeline_spec_);
   }
  
  const gchar *
  GstParseToBinSrc::get_gst_launch_pipeline (void *user_data)
  {
    GstParseToBinSrc *context = static_cast <GstParseToBinSrc *> (user_data);
    return context->gst_launch_pipeline_;
  }

  bool 
  GstParseToBinSrc::clean ()
  {
    clear_shmdatas ();
    reset_bin ();//bool res = unregister_shmdata (make_file_name ("video"));
    return true;
  }

  bool 
  GstParseToBinSrc::start ()
  {
    clean ();
    if (! to_shmdata ())
      return false;
    uninstall_property ("gst-pipeline");
    return true;
  }
  
  bool 
  GstParseToBinSrc::stop ()
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
