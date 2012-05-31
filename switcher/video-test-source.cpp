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

#include "switcher/video-test-source.h"
#include <gst/gst.h>

namespace switcher
{

  VideoTestSource::VideoTestSource ()
  {
    g_print ("video test source constructor \n");
    videotestsrc_ = gst_element_factory_make ("videotestsrc",NULL);
    //set the name before registering properties
    name_ = gst_element_get_name (videotestsrc_);

    // //This register all the properties
    // guint numproperty;
    // GParamSpec **property = g_object_class_list_properties (G_OBJECT_GET_CLASS(videotestsrc_), &numproperty);
    // for (guint i = 0; i < numproperty; i++) {
    //   register_property (G_OBJECT (videotestsrc_),property[i]);
    //   //Property *prop = new Property (G_OBJECT (videotestsrc_),property[i]);
    //   //prop->print();
    // }

    //registering "pattern" and "is-live" properties 
    // GParamSpec *pspec_pattern = g_object_class_find_property (G_OBJECT_GET_CLASS(videotestsrc_), "pattern");
    // if (pspec_pattern != NULL)
    //   {
    // 	register_property (G_OBJECT (videotestsrc_), pspec_pattern);
    //   }

    // GParamSpec *pspec_islive = g_object_class_find_property (G_OBJECT_GET_CLASS(videotestsrc_), "is-live");
    // if (pspec_islive != NULL)
    //   {
    //  	register_property (G_OBJECT (videotestsrc_), pspec_islive);
    //   }
    
    //registering "pattern" and "is-live" properties 
    register_property (G_OBJECT (videotestsrc_),"pattern");
    register_property (G_OBJECT (videotestsrc_),"is-live");
    
    set_raw_video_element (videotestsrc_);
  }

}
