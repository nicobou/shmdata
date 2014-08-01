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

#include "single-pad-gst-sink.h"
#include "gst-utils.h"
#include <utility>

namespace switcher
{
  SinglePadGstSink::~SinglePadGstSink ()
  {}  
  
  SinglePadGstSink::SinglePadGstSink () 
  {
    install_connect_method (std::bind (&SinglePadGstSink::connect,
				       this, 
				       std::placeholders::_1),
			    nullptr, //no disconnect
			    std::bind (&SinglePadGstSink::disconnect_all,
				       this),
			    std::bind (&SinglePadGstSink::can_sink_caps,
				       this, 
				       std::placeholders::_1), 
			    1);
  }
  
  bool
  SinglePadGstSink::disconnect_all ()
  {
    on_shmdata_disconnect ();
    return true;
  }

  bool
  SinglePadGstSink::connect (std::string shmdata_socket_path)
  {
    unregister_shmdata (shmdata_socket_path);
    on_shmdata_connect (shmdata_socket_path);

    ShmdataReader::ptr reader_;
    reader_.reset (new ShmdataReader ());
    reader_->set_path (shmdata_socket_path.c_str());
    shmdata_path_= shmdata_socket_path;
    reader_->set_g_main_context (get_g_main_context ());
    reader_->set_bin (bin_);

    if (sink_element_ != nullptr)
      reader_->set_sink_element (sink_element_);
    if (connection_hook_ != nullptr) 
      {
	g_debug ("SinglePadGstSink::connect set on_first_data_hook ");
	reader_->set_on_first_data_hook (connection_hook_, hook_user_data_);
      }
    reader_->start ();
    register_shmdata (reader_);
    return true;
  }

  void
  SinglePadGstSink::set_sink_element (GstElement *sink)
  {
    set_sink_element_no_connect (sink);
    if (!shmdata_path_.empty ())
      connect (shmdata_path_);
  }
 
  void
  SinglePadGstSink::set_sink_element_no_connect (GstElement *sink)
  {
    if (sink_element_ != nullptr && sink_element_ != sink)
      GstUtils::clean_element (sink_element_);
    //sink element will be added to bin_ by the shmdata reader when appropriate
    sink_element_ = sink;
  }

 
  void 
  SinglePadGstSink::set_on_first_data_hook (ShmdataReader::on_first_data_hook cb, void *user_data)
  {
    connection_hook_ = cb;
    hook_user_data_ = user_data;
  }

  bool
  SinglePadGstSink::can_sink_caps (std::string caps)
  {
    g_warning ("%s is not implemented for this quiddity");
    return false;
  }

  void 
  SinglePadGstSink::on_shmdata_connect (std::string shmdata_sochet_path) 
  {}

  void 
  SinglePadGstSink::on_shmdata_disconnect () 
  {}


}
