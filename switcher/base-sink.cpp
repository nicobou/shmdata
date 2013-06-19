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

#include "base-sink.h"
#include <utility>

namespace switcher
{
  BaseSink::~BaseSink ()
  {
    g_debug ("~BaseSink");
  }  
  
  BaseSink::BaseSink () :
    connection_hook_ (NULL)
  {
    
    reader_.reset (new ShmdataReader ());
    sink_element_ = NULL;
    
    //registering connect
    std::vector<GType> connect_arg_types;
    connect_arg_types.push_back (G_TYPE_STRING);
    register_method("connect",(void *)&BaseSink::connect_wrapped, connect_arg_types,(gpointer)this);
    std::vector<std::pair<std::string,std::string> > arg_desc;
    std::pair<std::string,std::string> socket;
    socket.first = "socket";
    socket.second = "socket path of the shmdata to connect to";
    arg_desc.push_back (socket); 
    if (!set_method_description ("connect", "connect the sink to a shmdata socket", arg_desc))
      g_warning ("base sink: cannot set method description for \"connect\"");

  }

   gboolean
   BaseSink::connect_wrapped (gpointer connector_name, gpointer user_data)
  {
    BaseSink *context = static_cast<BaseSink*>(user_data);
       
    if (context->connect ((char *)connector_name))
      return TRUE;
    else
      return FALSE;
  }

  bool
  BaseSink::connect (std::string shmdata_socket_path)
  {
    g_debug ("BaseSink::connect");
    reader_->set_path (shmdata_socket_path.c_str());
    reader_->set_g_main_context (get_g_main_context ());
    reader_->set_bin (bin_);
    if (sink_element_ !=NULL)
      reader_->set_sink_element (sink_element_);
    if (connection_hook_ != NULL) 
      {
	g_debug ("BaseSink::connect set on_first_data_hook ");
	reader_->set_on_first_data_hook (connection_hook_, hook_user_data_);
      }
    if (runtime_) // starting the reader if runtime is set
      reader_->start ();
    register_shmdata_reader (reader_);
    return true;
  }

  void
  BaseSink::set_sink_element (GstElement *sink)
  {
    //sink element will be added to bin_ by the shmdata reader when appropriate
    sink_element_ = sink;
  }

  void 
  BaseSink::set_on_first_data_hook (ShmdataReader::on_first_data_hook cb, void *user_data)
  {
    connection_hook_ = cb;
    hook_user_data_ = user_data;
  }

}
