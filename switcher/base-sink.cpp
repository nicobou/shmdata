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

#include "switcher/base-sink.h"

namespace switcher
{
  BaseSink::BaseSink ()
  {

    reader_.reset (new ShmdataReader ());
    
    //registering connect
    std::vector<GType> connect_arg_types;
    connect_arg_types.push_back (G_TYPE_STRING);
    register_method("connect",(void *)&BaseSink::connect_wrapped, connect_arg_types,(gpointer)this);
  }

   gboolean
   BaseSink::connect_wrapped (gpointer connector_name, gpointer user_data)
  {
    //std::string connector = static_cast<std::string>(connector_name);
    BaseSink *context = static_cast<BaseSink*>(user_data);
    
    if (context->connect ((char *)connector_name))
      return TRUE;
    else
      return FALSE;
  }

  bool
  BaseSink::connect (std::string shmdata_socket_path)
  {
    gst_bin_add (GST_BIN (bin_), sink_element_);
    reader_->plug (shmdata_socket_path.c_str(),bin_,sink_element_);
    return false;
  }

  void
  BaseSink::set_sink_element (GstElement *sink)
  {
    sink_element_ = sink;
  }

}
