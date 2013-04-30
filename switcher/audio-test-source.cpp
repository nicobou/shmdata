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

#include "audio-test-source.h"
#include <gst/gst.h>
#include "gst-utils.h"
namespace switcher
{

  QuiddityDocumentation AudioTestSource::doc_ ("audio source", "audiotestsrc", 
						     "Creates audio test signals");
  
  bool
  AudioTestSource::init ()
  {
    if (!GstUtils::make_element ("audiotestsrc",&audiotestsrc_))
      return false;

    g_object_set (G_OBJECT (audiotestsrc_),
		  "is-live", TRUE,
		  "samplesperbuffer",512,
		  NULL);


    //set the name before registering properties
    set_name (gst_element_get_name (audiotestsrc_));
    
    //registering 
    register_property (G_OBJECT (audiotestsrc_),"samplesperbuffer","samplesperbuffer");
    register_property (G_OBJECT (audiotestsrc_),"wave","wave");
    register_property (G_OBJECT (audiotestsrc_),"freq","freq");
    register_property (G_OBJECT (audiotestsrc_),"volume","volume");

    set_raw_audio_element (audiotestsrc_);
    return true;
  }

  AudioTestSource::~AudioTestSource()
  {
    GstUtils::clean_element (audiotestsrc_);
  }

  QuiddityDocumentation 
  AudioTestSource::get_documentation ()
  {
    return doc_;
  }
  
}
