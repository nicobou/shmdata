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

#include "v4l2src.h"
#include "switcher/gst-utils.h"
#include <cstdlib>  // For srand() and rand()
#include <ctime>    // For time()

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(V4L2Src,
				       "Video Capture (with v4l2)",
				       "video capture source", 
				       "Discover and use v4l2 supported capture cards and cameras",
				       "GPL",
				       "v4l2src",				
				       "Nicolas Bouillot");

  bool
  V4L2Src::init ()
  {

    if (!GstUtils::make_element ("v4l2src",&v4l2src_))
      return false;
    
    //set the name before registering properties
    set_name (gst_element_get_name (v4l2src_));

    //registering "pattern"
    register_property (G_OBJECT (v4l2src_),"device","device", "Path To The Device");
    register_property (G_OBJECT (v4l2src_),"brightness","brightness", "Brightness");
    register_property (G_OBJECT (v4l2src_),"contrast","contrast", "Contrast");
    register_property (G_OBJECT (v4l2src_),"saturation","saturation", "Saturation");
    register_property (G_OBJECT (v4l2src_),"hue","hue", "Hue");
    register_property (G_OBJECT (v4l2src_),"norm","norm", "Video Standard");

    //registering capture
    register_method("capture",
		    (void *)&capture_wrapped, 
		    Method::make_arg_type_description (G_TYPE_BOOLEAN, NULL),
		    (gpointer)this);
    set_method_description ("capture", 
			    "start or stop capture", 
			    Method::make_arg_description ("capturing", 
							  "true for starting the capture, false for stoping",
							  NULL));
    
    //capture (true);
    
    return true;
  }

  bool 
  V4L2Src::capture (gboolean capture)
  {
    g_print ("coucou1\n");
    if (!capture)
      return false;
    g_print ("coucou2\n");
    
    set_raw_video_element (v4l2src_);

    GstUtils::sync_state_with_parent (bin_);
    GstUtils::wait_state_changed (bin_);
    g_print ("coucou3\n");
    return true;
  }

  gboolean 
  V4L2Src::capture_wrapped (gboolean capture, gpointer user_data)
  {
    V4L2Src *context = static_cast<V4L2Src *>(user_data);

    if (context->capture (capture))
      return TRUE;
    else
      return FALSE;
    
  }

}
