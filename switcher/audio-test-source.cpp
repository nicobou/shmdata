/*
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
  
  AudioTestSource::AudioTestSource () :
    audiotestsrc_ (NULL)
  {}
  
  bool
  AudioTestSource::init_segment ()
  {
    init_startable (this);
    return make_audiotestsrc ();
  }
  
  bool 
  AudioTestSource::make_audiotestsrc ()
  {
    uninstall_property ("volume");
    uninstall_property ("freq");
    uninstall_property ("samplesperbuffer");
    uninstall_property ("wave");

    GstElement *audiotest;
    if (!GstUtils::make_element ("audiotestsrc",&audiotest))
      return false;

    g_object_set (G_OBJECT (audiotest),
		  "is-live", TRUE,
		  NULL);

    if (audiotestsrc_ != NULL)
      {
	GstUtils::apply_property_value (G_OBJECT (audiotestsrc_), G_OBJECT (audiotest), "volume");
	GstUtils::apply_property_value (G_OBJECT (audiotestsrc_), G_OBJECT (audiotest), "freq");
	GstUtils::apply_property_value (G_OBJECT (audiotestsrc_), G_OBJECT (audiotest), "samplesperbuffer");
	GstUtils::apply_property_value (G_OBJECT (audiotestsrc_), G_OBJECT (audiotest), "wave");

	GstUtils::clean_element (audiotestsrc_);
      }
    else
      g_object_set (G_OBJECT (audiotest),
		    "samplesperbuffer", 512,
		    NULL);

    audiotestsrc_ = audiotest;

    //registering 
    install_property (G_OBJECT (audiotestsrc_),
		       "volume",
		       "volume", 
		       "Volume");
    install_property (G_OBJECT (audiotestsrc_),
		       "freq",
		       "freq", 
		       "Frequency");
    // install_property (G_OBJECT (audiotestsrc_),
    // 		       "samplesperbuffer",
    // 		       "samplesperbuffer", 
    // 		       "Samples Per Buffer");
    install_property (G_OBJECT (audiotestsrc_),
		       "wave", 
		       "wave", 
		       "Signal Form");

    return true;
  }

  AudioTestSource::~AudioTestSource()
  {
    GstUtils::clean_element (audiotestsrc_);
  }
  
  bool 
  AudioTestSource::start ()
  {
    set_raw_audio_element (audiotestsrc_);
    return true;
  }

  bool 
  AudioTestSource::stop ()
  {
    make_audiotestsrc ();
    unset_raw_audio_element ();
    return true;
  }

}
