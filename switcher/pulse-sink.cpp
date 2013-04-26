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

#include "pulse-sink.h"
#include "gst-utils.h"

namespace switcher
{

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

  QuiddityDocumentation PulseSink::doc_ ("audio sink", "pulsesink",
					 "Output sound to pulse server");

  QuiddityDocumentation 
  PulseSink::get_documentation ()
  {
    return doc_;
  }
  
}
