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

#include "switcher/xvimagesink.h"

namespace switcher
{

  const QuiddityDocumentation Xvimagesink::doc_ ("video sink", "xvimagesink",
						 "Video window with minimal features");
  
  
  bool
  Xvimagesink::init ()
  {
    xvimagesink_ = gst_element_factory_make ("xvimagesink",NULL);
    
    //set the name before registering properties
    set_name (gst_element_get_name (xvimagesink_));
    g_object_set (G_OBJECT (xvimagesink_), "sync", FALSE, NULL);

    //registering "sync"
    //register_property (G_OBJECT (xvimagesink_),"sync","videosink");
    
    set_sink_element (xvimagesink_);

    return true;
  }
  


  QuiddityDocumentation 
  Xvimagesink::get_documentation ()
  {
    return doc_;
  }
  
}
