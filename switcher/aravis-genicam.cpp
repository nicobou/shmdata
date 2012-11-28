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

#include "switcher/aravis-genicam.h"

namespace switcher
{

  const QuiddityDocumentation AravisGenicam::doc_ ("video source", "genicam",
						   "Genicam video source using the Aravis library");
  
  
  bool
  AravisGenicam::init ()
  {
    aravissrc_ = gst_element_factory_make ("aravissrc",NULL);
    if (aravissrc_ == NULL)
      {
	g_error ("aravissrc not available, install aravis (http://git.gnome.org/browse/aravis/)");
	return false;
      }
    set_name (gst_element_get_name (aravissrc_));
    
    register_property (G_OBJECT (aravissrc_),"gain","aravissrc");

    register_property (G_OBJECT (aravissrc_),"gain-auto","aravissrc");

    register_property (G_OBJECT (aravissrc_),"exposure","aravissrc");
    register_property (G_OBJECT (aravissrc_),"exposure-auto","aravissrc");
    register_property (G_OBJECT (aravissrc_),"h-binning","aravissrc");
    register_property (G_OBJECT (aravissrc_),"v-binning","aravissrc");
    register_property (G_OBJECT (aravissrc_),"offset-x","aravissrc");
    register_property (G_OBJECT (aravissrc_),"offset-y","aravissrc");

    set_raw_video_element (aravissrc_);
    return true;
  }
  


  QuiddityDocumentation 
  AravisGenicam::get_documentation ()
  {
    return doc_;
  }
  
}
