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

#include "switcher/src-connector.h"

namespace switcher
{

  SrcConnector::SrcConnector () :
    tee_ (gst_element_factory_make ("tee",NULL))
  {

    GstElement *xvimagesink = gst_element_factory_make ("xvimagesink",NULL);
    GstElement *xvimagesink_queue = gst_element_factory_make ("queue",NULL);
    GstElement *fakesink = gst_element_factory_make ("fakesink",NULL);
    GstElement *fakesink_queue = gst_element_factory_make ("queue",NULL);

    gst_bin_add_many (GST_BIN (bin_),
		      tee_,
		      //fakesink_queue,
		      //fakesink,
		      xvimagesink_queue,
		      xvimagesink,
		      NULL);
    //gst_element_link_many (tee_, fakesink_queue, fakesink, NULL);
    gst_element_link_many (tee_, xvimagesink_queue, xvimagesink, NULL);
    //g_object_set (G_OBJECT (xvimagesink),
		  // "sync", 
		  // FALSE, 
		  // NULL);
  }
  
  


}  // end of namespace
