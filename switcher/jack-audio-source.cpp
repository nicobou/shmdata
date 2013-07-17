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

#include "jack-audio-source.h"
#include <gst/gst.h>
#include "gst-utils.h"
namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(JackAudioSource,
				       "Jack Audio",
				       "audio source", 
				       "get audio from jack",
				       "GPL",
				       "jackaudiosrc", 
				       "Nicolas Bouillot");
  
  bool
  JackAudioSource::init ()
  {
    if (!GstUtils::make_element ("jackaudiosrc",&jackaudiosrc_))
      return false;

    // g_object_set (G_OBJECT (jackaudiosrc_),
    // 		  "is-live", TRUE,
    // 		  "samplesperbuffer",512,
    // 		  NULL);


    //set the name before registering properties
    set_name (gst_element_get_name (jackaudiosrc_));
    
    //registering 
    // register_property (G_OBJECT (jackaudiosrc_),"samplesperbuffer","samplesperbuffer");
    // register_property (G_OBJECT (jackaudiosrc_),"wave","wave");
    // register_property (G_OBJECT (jackaudiosrc_),"freq","freq");
    // register_property (G_OBJECT (jackaudiosrc_),"volume","volume");

    set_raw_audio_element (jackaudiosrc_);
    return true;
  }

  JackAudioSource::~JackAudioSource()
  {
    GstUtils::clean_element (jackaudiosrc_);
  }
  
}
