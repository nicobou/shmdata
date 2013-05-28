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

#include "fakesink.h"
#include "gst-utils.h"

namespace switcher
{

  QuiddityDocumentation FakeSink::doc_ ("fakesink sink", "fakesink",
					"fakesink for testing purpose");

  
  FakeSink::~FakeSink ()
  {
    g_debug ("~fakesink");
    GstUtils::clean_element (fakesink_);
  }
  
  bool
  FakeSink::init ()
  {
    if (!GstUtils::make_element ("fakesink", &fakesink_))
      return false;

    num_bytes_since_last_update_ = 0;

    //set the name before registering properties
    set_name (gst_element_get_name (fakesink_));
    g_object_set (G_OBJECT (fakesink_), 
		  "sync", FALSE, 
		  "signal-handoffs", TRUE,
		  NULL);

    g_signal_connect(fakesink_, "handoff", (GCallback)on_handoff_cb, this);

    //registering some properties 
    register_property (G_OBJECT (fakesink_),"last-message","last-message");
    
    byte_rate_prop_.reset (new CustomPropertyHelper ());
    byte_rate_spec_ = 
      byte_rate_prop_->make_int_property ("byte-rate", 
     					  "the byte rate (updated each second)",
     					  0,
     					  G_MAXINT,
     					  0,
     					  (GParamFlags) G_PARAM_READABLE,
     					  NULL,
     					  FakeSink::get_byte_rate,
     					  this);
    
    register_property_by_pspec (byte_rate_prop_->get_gobject (), 
     				byte_rate_spec_, 
     				"byte-rate");
    
    
    update_byterate_id_ = GstUtils::g_timeout_add_to_context (1000, 
     							      update_byte_rate, 
     							      this,
     							      get_g_main_context ());
    
    set_sink_element (fakesink_);
    return true;
  }
  
  gboolean 
  FakeSink::update_byte_rate (gpointer user_data) 
  {
    FakeSink *context = static_cast <FakeSink *> (user_data);
    if (context == NULL) //auto cleanup
      return FALSE;
    context->byte_rate_ = context->num_bytes_since_last_update_;
    context->num_bytes_since_last_update_ = 0;
    context->byte_rate_prop_->notify_property_changed (context->byte_rate_spec_);
    return TRUE;
  }

  void 
  FakeSink::on_handoff_cb (GstElement* object,
			   GstBuffer* buf,
			   GstPad* pad,
			   gpointer user_data)
  {
    FakeSink *context = static_cast <FakeSink *> (user_data);
    context->num_bytes_since_last_update_ += GST_BUFFER_SIZE (buf);
  }

  gint
  FakeSink::get_byte_rate (void *user_data)
  {
    FakeSink *context = static_cast<FakeSink *> (user_data);
    return context->byte_rate_;
  }

  
  QuiddityDocumentation 
  FakeSink::get_documentation ()
  {
    return doc_;
  }
  
}
