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

#include "switcher/gconf-video-source.h"
#include <gst/gst.h>

namespace switcher
{

  GconfVideoSource::GconfVideoSource ()
  {
    
    GstElement *bin = gst_element_factory_make ("bin", NULL);

    gconfvideosource_ = gst_element_factory_make ("autovideosrc",NULL);
    //g_object_set (G_OBJECT (gconfvideosource_), "is-live", TRUE, NULL);
    
    videorate_ = gst_element_factory_make ("videorate",NULL);

    //CAPS
    capsfilter_ = gst_element_factory_make ("capsfilter",NULL);
    caps_ =     gst_caps_new_simple ("video/x-raw-yuv",
				     // "format", GST_TYPE_FOURCC,
				     // GST_MAKE_FOURCC ('A', 'Y', 'U', 'V'),
				     //"format", GST_TYPE_FOURCC,
				     //  GST_MAKE_FOURCC ('I', '4', '2', '0'),
				     "framerate", GST_TYPE_FRACTION, 10, 1,
				     // "pixel-aspect-ratio", GST_TYPE_FRACTION, 1, 1, 
				     //  "width", G_TYPE_INT, 640, 
				     //  "height", G_TYPE_INT, 480,
				     NULL);
    g_object_set (G_OBJECT (capsfilter_), "caps", caps_, NULL);
        
    gst_bin_add_many (GST_BIN (bin),
		      gconfvideosource_,
		      videorate_,
		      capsfilter_,
		      NULL);
    gst_element_link_many (gconfvideosource_,
			   videorate_,
			   capsfilter_,
			   NULL);
    
    GstPad *sinkpad = gst_element_get_static_pad(capsfilter_, "src"); 
    GstPad *ghost_sinkpad = gst_ghost_pad_new (NULL, sinkpad);
    gst_element_add_pad (bin, ghost_sinkpad);
    gst_object_unref (sinkpad);
    
    //set the name
    name_ = gst_element_get_name (gconfvideosource_);

    set_raw_video_element (bin);
  }


}
