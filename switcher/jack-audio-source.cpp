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
				       "test", 
				       "get audio from jack",
				       "LGPL",
				       "jackaudiosrc", 
				       "Nicolas Bouillot");
  
  JackAudioSource::JackAudioSource() :
    jackaudiosrc_ (NULL),
    audioconvert_ (NULL),
    capsfilter_ (NULL),
    jackaudiosrc_bin_ (NULL)
  {}

  bool
  JackAudioSource::init_segment ()
  {
    if (false == make_elements ())
      return false;
    init_startable (this);

    // g_object_set (G_OBJECT (jackaudiosrc_),
    // 		  "is-live", TRUE,
    // 		  "samplesperbuffer",512,
    // 		  NULL);

    return true;
  }

  JackAudioSource::~JackAudioSource()
  {}
  
  bool 
  JackAudioSource::start ()
  {
    make_elements ();
    set_raw_audio_element (jackaudiosrc_bin_);
    return true;
  }
  
  bool 
  JackAudioSource::stop ()
  {
    reset_bin ();
    return true;
  }

  bool
  JackAudioSource::make_elements ()
  {
    if (!GstUtils::make_element ("jackaudiosrc",&jackaudiosrc_))
      return false;
    if (!GstUtils::make_element ("audioconvert",&audioconvert_))
      return false;
    if (!GstUtils::make_element ("capsfilter",&capsfilter_))
      return false;
    if (!GstUtils::make_element ("bin",&jackaudiosrc_bin_))
      return false;

    GstCaps *caps = gst_caps_new_simple ("audio/x-raw-int",
					 "width", G_TYPE_INT, 16,
					 NULL);
    g_object_set (G_OBJECT (capsfilter_), "caps", caps,NULL);
    gst_caps_unref(caps);

    gst_bin_add_many (GST_BIN (jackaudiosrc_bin_),
		      jackaudiosrc_,
		      audioconvert_,
		      capsfilter_,
		      NULL);

    gst_element_link_many (jackaudiosrc_,
			   audioconvert_,
			   capsfilter_,
			   NULL);

    GstPad *src_pad = gst_element_get_static_pad (capsfilter_, "src");
    GstPad *ghost_srcpad = gst_ghost_pad_new (NULL, src_pad);
    gst_pad_set_active(ghost_srcpad,TRUE);
    gst_element_add_pad (jackaudiosrc_bin_, ghost_srcpad); 
    gst_object_unref (src_pad);

    return true;
  }


}
