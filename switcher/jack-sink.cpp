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

#include "jack-sink.h"
#include "gst-utils.h"
#include "quiddity-command.h"
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(JackSink, 
				       "Audio Display (with Jack Audio)",
				       "audio sink", 
				       "Audio display with minimal features",
				       "LGPL",
				       "jacksink",
				       "Nicolas Bouillot");
  
  bool
  JackSink::init_segment ()
  {
    if (false == make_elements ())
      return false;
    init_startable (this);
    
    client_name_spec_ =
      custom_props_->make_string_property ("jack-client-name", 
					   "the jack client name",
					   "switcher",
					   (GParamFlags) G_PARAM_READWRITE,
					   JackSink::set_client_name,
					   JackSink::get_client_name,
					   this);
    install_property_by_pspec (custom_props_->get_gobject (), 
			       client_name_spec_, 
			       "client-name",
			       "Client Name");
    

    return true;
  }
  
  JackSink::JackSink () :
    jacksink_ (NULL),
    custom_props_ (new CustomPropertyHelper ()),
    client_name_spec_ (NULL),
    client_name_ (g_strdup ("switcher"))
  {}

  JackSink::~JackSink ()
  {
    if (NULL != client_name_)
      g_free (client_name_);
  }
  
  bool 
  JackSink::make_elements ()
  {
    GError *error = NULL;

    gchar *description = g_strdup_printf ("audioconvert ! jackaudiosink client-name=%s sync=false", client_name_);
    
    jacksink_ = gst_parse_bin_from_description (description,
						TRUE,
						&error);
    g_object_set (G_OBJECT (jacksink_), "async-handling",TRUE, NULL);
    g_free (description);

    if (error != NULL)
      {
	g_warning ("%s",error->message);
	g_error_free (error);
	return false;
      }
    return true;
  }
 
  bool 
  JackSink::start ()
  {
    if (false == make_elements ())
      return false;
    set_sink_element (jacksink_);
    return true;
  }
  
  bool 
  JackSink::stop ()
  {
    reset_bin ();
    return true;
  }

  void 
  JackSink::set_client_name (const gchar *value, void *user_data)
  {
    JackSink *context = static_cast <JackSink *> (user_data);
    if (NULL != context->client_name_)
      g_free (context->client_name_);
    context->client_name_ = g_strdup(value);
    context->custom_props_->notify_property_changed (context->client_name_spec_);
   }
  
  gchar *
  JackSink::get_client_name (void *user_data)
  {
    JackSink *context = static_cast <JackSink *> (user_data);
    return context->client_name_;
  }
}
