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

#include "switcher/connector.h"

namespace switcher
{

  Connector::Connector ()
  {
    //g_print ("connector constructor \n");
    tee_ = gst_element_factory_make ("tee",NULL);
    gst_bin_add (GST_BIN (bin_),tee_);
  }
  
  GstElement *
  Connector::get_src_element ()
  {
    return tee_;
  }

  GstElement *
  Connector::get_sink_element ()
  {
    return tee_;
  }


}  // end of namespace
