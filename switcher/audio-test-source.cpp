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

#include "audio-test-source.h"
#include <gst/gst.h>
#include "gst-utils.h"
namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(AudioTestSource,
				       "Audio Test",
				       "audio source", 
				       "Creates audio test signals",
				       "LGPL",
				       "audiotestsrc", 
				       "Nicolas Bouillot");
  
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
    register_property (G_OBJECT (audiotestsrc_),"volume","volume", "Volume", true, true);
    register_property (G_OBJECT (audiotestsrc_),"freq","freq", "Frequency", true, true);
    register_property (G_OBJECT (audiotestsrc_),
		       "samplesperbuffer",
		       "samplesperbuffer", 
		       "Samples Per Buffer", 
		       true, 
		       true);
    register_property (G_OBJECT (audiotestsrc_),"wave","wave", "Signal Form", true, true);

    set_raw_audio_element (audiotestsrc_);
    return true;
  }

  AudioTestSource::~AudioTestSource()
  {
    GstUtils::clean_element (audiotestsrc_);
  }
  
}
