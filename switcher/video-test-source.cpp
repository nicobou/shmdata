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

#include "video-test-source.h"
#include <gst/gst.h>
#include "gst-utils.h"

namespace switcher
{
  QuiddityDocumentation VideoTestSource::doc_  ("video source", "videotestsrc",				
						"Creates a test video stream");

  VideoTestSource::~VideoTestSource()
  {
    GstUtils::clean_element (videotestsrc_);
  }
  
  bool
  VideoTestSource::init ()
  {
    if (!GstUtils::make_element ("videotestsrc",&videotestsrc_))
      return false;

   g_object_set (G_OBJECT (videotestsrc_),
		  "is-live", TRUE,
		  NULL);
    
    //set the name before registering properties
    set_name (gst_element_get_name (videotestsrc_));

    // //This register all the properties
    // guint numproperty;
    // GParamSpec **property = g_object_class_list_properties (G_OBJECT_GET_CLASS(videotestsrc_), &numproperty);
    // for (guint i = 0; i < numproperty; i++) {
    //   register_property (G_OBJECT (videotestsrc_),property[i]);
    //   //Property *prop = new Property (G_OBJECT (videotestsrc_),property[i]);
    //   //prop->print();
    // }

    //registering "pattern"
    register_property (G_OBJECT (videotestsrc_),"pattern","pattern");
    
    set_raw_video_element (videotestsrc_);
    return true;
  }
  
  
  QuiddityDocumentation 
  VideoTestSource::get_documentation ()
  {
    return doc_;
  }
  
}
