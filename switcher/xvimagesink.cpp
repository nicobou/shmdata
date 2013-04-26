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

#include "xvimagesink.h"
#include "gst-utils.h"
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

namespace switcher
{

  QuiddityDocumentation Xvimagesink::doc_ ("video sink", "videosink",
					   "Video window with minimal features");
  
  
  bool
  Xvimagesink::init ()
  {
#if HAVE_OSX
    if (!GstUtils::make_element ("osxvideosink", &xvimagesink_))
      return false;
#else
    if (!GstUtils::make_element ("xvimagesink", &xvimagesink_))
      return false;
#endif

    //set the name before registering properties
    set_name (gst_element_get_name (xvimagesink_));
    g_object_set (G_OBJECT (xvimagesink_), "sync", FALSE, NULL);

    //registering "sync"
    //register_property (G_OBJECT (xvimagesink_),"sync","sync");
    
    set_sink_element (xvimagesink_);

    return true;
  }
  


  QuiddityDocumentation 
  Xvimagesink::get_documentation ()
  {
    return doc_;
  }
  
}
