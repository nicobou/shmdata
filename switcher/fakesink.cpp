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

    //set the name before registering properties
    set_name (gst_element_get_name (fakesink_));
    g_object_set (G_OBJECT (fakesink_), 
		  "sync", FALSE, 
		  "enable-last-buffer", TRUE, 
		  NULL);

    //registering some properties 
    register_property (G_OBJECT (fakesink_),"last-message","last-message");
    register_property (G_OBJECT (fakesink_),"last-buffer","last-buffer");
    
    set_sink_element (fakesink_);
    return true;
  }
  


  QuiddityDocumentation 
  FakeSink::get_documentation ()
  {
    return doc_;
  }
  
}
