/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "base-sink.h"
#include "gst-utils.h"
#include <utility>

namespace switcher
{
  BaseSink::~BaseSink ()
  {
    g_debug ("~BaseSink");
  }  
  
  BaseSink::BaseSink () :
    connection_hook_ (NULL),
    hook_user_data_ (NULL),
    sink_element_ (),
    shmdata_path_ ("")
  {
    //registering connect
    install_method ("Connect",
		    "connect",
		    "connect the sink to a shmdata socket", 
		    "success or fail",
		    Method::make_arg_description ("Shmdata Path",
						  "socket",
						  "shmdata socket path to connect with",
						  NULL),
		    (Method::method_ptr)&connect_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    this);
  
    //registering disconnect
    install_method ("Disconnect",
		    "disconnect",
		    "disconnect the sink from the shmdata socket", 
		    "success or fail",
		    Method::make_arg_description ("none",
						  NULL),
		    (Method::method_ptr)&disconnect, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_NONE, NULL),
		    this);
  }
  
  gboolean
  BaseSink::disconnect (gpointer /*unused*/, gpointer user_data)
  {
    //std::string connector = static_cast<std::string>(connector_name);
    BaseSink *context = static_cast<BaseSink*>(user_data);
    context->clear_shmdatas ();
    context->on_shmdata_disconnect ();
    return TRUE;
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
    unregister_shmdata_reader (shmdata_socket_path);
    on_shmdata_connect (shmdata_socket_path);

    ShmdataReader::ptr reader_;
    reader_.reset (new ShmdataReader ());
    reader_->set_path (shmdata_socket_path.c_str());
    shmdata_path_= shmdata_socket_path;
    reader_->set_g_main_context (get_g_main_context ());
    reader_->set_bin (bin_);

    if (sink_element_ !=NULL)
      reader_->set_sink_element (sink_element_);
    if (connection_hook_ != NULL) 
      {
	g_debug ("BaseSink::connect set on_first_data_hook ");
	reader_->set_on_first_data_hook (connection_hook_, hook_user_data_);
      }
    reader_->start ();
    register_shmdata_reader (reader_);
    return true;
  }

  void
  BaseSink::set_sink_element (GstElement *sink)
  {
    set_sink_element_no_connect (sink);
    if (g_strcmp0 (shmdata_path_.c_str (), "") != 0)
      connect (shmdata_path_);
  }
 
  void
  BaseSink::set_sink_element_no_connect (GstElement *sink)
  {
    if (sink_element_ != NULL && sink_element_ != sink)
      GstUtils::clean_element (sink_element_);
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
