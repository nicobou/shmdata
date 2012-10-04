/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#include "switcher/audio-test-source.h"
#include <gst/gst.h>

namespace switcher
{

  QuiddityDocumentation AudioTestSource::doc_ ("audio source", "audiotestsrc", 
						 "Creates audio test signals");
  
  AudioTestSource::AudioTestSource ()
  {
    make_audiotestsource ();
  }

  AudioTestSource::AudioTestSource (QuiddityLifeManager::ptr life_manager)
  {
    life_manager_ = life_manager;
    make_audiotestsource ();
  }

  void
  AudioTestSource::make_audiotestsource ()
  {
    audiotestsrc_ = gst_element_factory_make ("audiotestsrc",NULL);
    g_object_set (G_OBJECT (audiotestsrc_),
		  "is-live", TRUE,
		  NULL);

    //set the name before registering properties
    name_ = gst_element_get_name (audiotestsrc_);
    
    //registering "pattern"
    register_property (G_OBJECT (audiotestsrc_),"samplesperbuffer","audiotestsrc");
    register_property (G_OBJECT (audiotestsrc_),"wave","audiotestsrc");
    register_property (G_OBJECT (audiotestsrc_),"freq","audiotestsrc");
    register_property (G_OBJECT (audiotestsrc_),"volume","audiotestsrc");

    set_raw_audio_element (audiotestsrc_);
  }

  QuiddityDocumentation 
  AudioTestSource::get_documentation ()
  {
    return doc_;
  }
  
}
