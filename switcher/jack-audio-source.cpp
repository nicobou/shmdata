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

#include "jack-audio-source.h"
#include <gst/gst.h>
#include "gst-utils.h"
namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(JackAudioSource,
				       "Jack Audio",
				       "audio source", 
				       "get audio from jack",
				       "LGPL",
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
    
    set_raw_audio_element (jackaudiosrc_);
    return true;
  }

  JackAudioSource::~JackAudioSource()
  {
    GstUtils::clean_element (jackaudiosrc_);
  }
  
}
