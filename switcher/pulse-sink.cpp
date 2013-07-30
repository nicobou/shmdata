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

#include "pulse-sink.h"
#include "gst-utils.h"

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PulseSink,
				       "PulseAudio Display",
				       "audio sink", 
				       "Output sound to pulse server",
				       "GPL",
				       "pulsesink",
				       "Nicolas Bouillot");

  bool 
  PulseSink::init ()
  {
    pulse_sink_ = gst_parse_bin_from_description ("audioconvert ! pulsesink sync=false",
						  TRUE,
						  NULL);
    
    //set the name before registering properties
    set_name (gst_element_get_name (pulse_sink_));
    
    set_sink_element (pulse_sink_);
    return true;
  }
  
  PulseSink::~PulseSink ()
  {
    GstUtils::clean_element (pulse_sink_);
  }

}
