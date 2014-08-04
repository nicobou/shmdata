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

#include "fakesink.h"
#include "gst-utils.h"

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(FakeSink,
				       "Shmdata Inspector",
				       "monitor", 
				       "fakesink for testing purpose",
				       "LGPL",
				       "fakesink",
				       "Nicolas Bouillot");

  FakeSink::FakeSink () :
    fakesink_ (nullptr),
    num_bytes_since_last_update_ (0),
    update_byterate_source_ (nullptr),
    byte_rate_ (0),
    string_caps_ (g_strdup ("unknown")),
    set_string_caps_ (true),
    props_ (new CustomPropertyHelper ()),
    byte_rate_spec_ (nullptr),
    caps_spec_ (nullptr)
    {} 
 
  FakeSink::~FakeSink ()
  {
     if (update_byterate_source_ != nullptr)
       g_source_destroy (update_byterate_source_);
    reset_bin ();
    GstUtils::clean_element (fakesink_);
    g_free (string_caps_);
  }
  
  bool
  FakeSink::init_gpipe ()
  {
    if (!GstUtils::make_element ("fakesink", &fakesink_))
      return false;

    g_object_set (G_OBJECT (fakesink_), 
		  "sync", FALSE, 
		  "signal-handoffs", TRUE,
		  nullptr);

    g_signal_connect (fakesink_, "handoff", (GCallback)on_handoff_cb, this);

    //registering some properties 
    //install_property (G_OBJECT (fakesink_),"last-message","last-message", "Last Message");
    
    byte_rate_spec_ = 
      props_->make_int_property ("byte-rate", 
     					  "the byte rate (updated each second)",
     					  0,
     					  G_MAXINT,
     					  byte_rate_,
     					  (GParamFlags) G_PARAM_READABLE,
     					  nullptr,
     					  FakeSink::get_byte_rate,
     					  this);
    
    install_property_by_pspec (props_->get_gobject (), 
     				byte_rate_spec_, 
     				"byte-rate",
				"Byte Rate (Bps)");
     
    update_byterate_source_ = GstUtils::g_timeout_add_to_context (1000, 
								  update_byte_rate, 
								  this,
								  get_g_main_context ());
    
    caps_spec_ = 
      props_->make_string_property ("caps", 
				    "caps of the attached shmdata",
				    "unknown",
				    (GParamFlags) G_PARAM_READABLE,
				    nullptr,
				    FakeSink::get_caps,
				    this);
    
    install_property_by_pspec (props_->get_gobject (), 
     				caps_spec_, 
     				"caps",
				"Capabilities");
    
    set_sink_element (fakesink_);
    return true;
  }
  
  gboolean 
  FakeSink::update_byte_rate (gpointer user_data) 
  {
    FakeSink *context = static_cast <FakeSink *> (user_data);
    context->byte_rate_ = context->num_bytes_since_last_update_;
    context->num_bytes_since_last_update_ = 0;
    context->props_->notify_property_changed (context->byte_rate_spec_);
    return TRUE;
  }

  void 
  FakeSink::on_handoff_cb (GstElement* /*object*/,
			   GstBuffer* buf,
			   GstPad* pad,
			   gpointer user_data)
  {
    FakeSink *context = static_cast <FakeSink *> (user_data);

    if (context->set_string_caps_)
      {
	context->set_string_caps_ = false; 
	GstCaps *caps = gst_pad_get_negotiated_caps (pad);
	g_free (context->string_caps_);
	context->string_caps_ = gst_caps_to_string (caps);
	context->props_->notify_property_changed (context->caps_spec_);
	gst_caps_unref (caps);
      }
    context->num_bytes_since_last_update_ += GST_BUFFER_SIZE (buf);
  }

  gint
  FakeSink::get_byte_rate (void *user_data)
  {
    FakeSink *context = static_cast<FakeSink *> (user_data);
    return context->byte_rate_;
  }

  const gchar *
  FakeSink::get_caps (void *user_data)
  {
    FakeSink *context = static_cast<FakeSink *> (user_data);
    return context->string_caps_;
  }
 
}
