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

#include "video-test-source.h"
#include <gst/gst.h>
#include "gst-utils.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(VideoTestSource,
				       "Video Test",
				       "video source", 
				       "Creates a test video stream",
				       "LGPL",
				       "videotestsrc",				
				       "Nicolas Bouillot");

  VideoTestSource::VideoTestSource() :
    videotestsrc_ (nullptr)
  {}
 
  VideoTestSource::~VideoTestSource()
  {
    GstUtils::clean_element (videotestsrc_);
  }
  
  bool
  VideoTestSource::init_gpipe ()
  {
    bool made = make_video_source (&videotestsrc_);    
    if (!made)
      return false;
    
    //"pattern" property available atfer initialization 
    install_property (G_OBJECT (videotestsrc_),
		      "pattern",
		      "pattern", 
		      "Video Pattern");
    return true;
  }

  bool
  VideoTestSource::make_video_source (GstElement **new_element)
  {

    GstElement *videotest;
    if (!GstUtils::make_element ("videotestsrc",&videotest))
      return false;
    
    g_object_set (G_OBJECT (videotest),
		  "is-live", TRUE,
		  nullptr);
    
    if (videotestsrc_ != nullptr)
      {
	GstUtils::apply_property_value (G_OBJECT (videotestsrc_), G_OBJECT (videotest), "pattern");
	
	GstUtils::clean_element (videotestsrc_);
      }
     
    videotestsrc_ = videotest;
    *new_element = videotestsrc_;

    return true;
  }

  bool 
  VideoTestSource::on_start ()
  {
    reinstall_property (G_OBJECT (videotestsrc_),
			"pattern",
			"pattern", 
			"Video Pattern");
    return true;
  }
  
  bool 
  VideoTestSource::on_stop ()
  {
    //need a new element for property setting 
    bool made = make_video_source (&videotestsrc_);    
    if (!made)
      return false;
    
    reinstall_property (G_OBJECT (videotestsrc_),
		       "pattern",
		       "pattern", 
		       "Video Pattern");
    return true;
  }
  
}
