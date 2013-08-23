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
  JackSink::init ()
  {
    GError *error = NULL;
    jacksink_ = gst_parse_bin_from_description ("audioconvert ! jackaudiosink sync=false",
						TRUE,
						&error);
    g_object_set (G_OBJECT (jacksink_), "async-handling",TRUE, NULL);

    if (error != NULL)
      {
	g_warning ("%s",error->message);
	g_error_free (error);
	return false;
      }
    
    set_sink_element (jacksink_);
    return true;
  }
  
  JackSink::JackSink ()
  {
  }
 
}
