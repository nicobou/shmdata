/*
 * This file is part of switcher-osc.
 *
 * switcher-osc is free software; you can redistribute it and/or
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

#include "switcher/json-builder.h"
#include "shmdata-to-osc.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ShmdataToOsc,
				       "shmdata to OSC messages (default to localhost:1056)",
				       "osc sink", 
				       "OSCprop reveives OSC messages and updates associated property",
				       "LGPL",
				       "shmOSC",
				       "Nicolas Bouillot");

  ShmdataToOsc::ShmdataToOsc () :
    custom_props_ (new CustomPropertyHelper ()),
    port_ (1056),
    host_ ("localhost"),
    port_spec_ (nullptr),
    host_spec_ (nullptr),
    address_ (nullptr),
    address_mutex_ ()
  {}
  
  bool
  ShmdataToOsc::init ()
  {
    init_startable (this);
    init_segment (this);
    
    install_connect_method (std::bind (&ShmdataToOsc::connect, this, std::placeholders::_1),
			    nullptr,
			    nullptr,
			    std::bind (&ShmdataToOsc::can_sink_caps, this, std::placeholders::_1),
			    1);//could be more but should be tested

    port_spec_ = 
      custom_props_->make_int_property ("Port", 
					"OSC destination port",
					1,
					65536,
					port_,
					(GParamFlags) G_PARAM_READWRITE,
					set_port,
					get_port,
					this);
    install_property_by_pspec (custom_props_->get_gobject (), 
			       port_spec_, 
			       "port",
			       "Port");
    host_spec_ = 
      custom_props_->make_string_property ("host", 
					   "destination host",
					   host_.c_str (),
					   (GParamFlags) G_PARAM_READWRITE,
					   ShmdataToOsc::set_host,
					   ShmdataToOsc::get_host,
					   this);
    install_property_by_pspec (custom_props_->get_gobject (), 
			       host_spec_, 
			       "host",
			       "Destination Host");
    return true;
  }

  ShmdataToOsc::~ShmdataToOsc ()
  {
    stop ();
  }

  void 
  ShmdataToOsc::set_port (const gint value, void *user_data)
  {
    ShmdataToOsc *context = static_cast <ShmdataToOsc *> (user_data);
    if (value == context->port_)
      return;
    context->stop ();
    context->port_ = value;
    context->start ();
    context->custom_props_->notify_property_changed (context->port_spec_);
  }
  
  gint 
  ShmdataToOsc::get_port (void *user_data)
  {
    ShmdataToOsc *context = static_cast <ShmdataToOsc *> (user_data);
    return context->port_;
  }

  bool 
  ShmdataToOsc::start ()
  {
    stop ();
    {
      std::unique_lock <std::mutex> lock (address_mutex_);
      address_ = lo_address_new (host_.c_str (), 
				 std::to_string (port_).c_str ());
    }
    if (nullptr == address_)
      return false;
    return true;
  }

  bool
  ShmdataToOsc::stop ()
  {
    if (nullptr != address_)
      {
 	std::unique_lock <std::mutex> lock (address_mutex_);
	lo_address_free (address_);
	address_ = nullptr;
      }
    return true;
  }

  void 
  ShmdataToOsc::set_host (const gchar *value, void *user_data)
  {
    ShmdataToOsc *context = static_cast <ShmdataToOsc *> (user_data);
    if (0 == context->host_.compare (value))
      return;
    context->stop ();
    context->host_ = value;
    context->start ();
    context->custom_props_->notify_property_changed (context->host_spec_);
  }
  
  const gchar *
  ShmdataToOsc::get_host (void *user_data)
  {
    ShmdataToOsc *context = static_cast <ShmdataToOsc *> (user_data);
    return context->host_.c_str ();
  }

  bool 
  ShmdataToOsc::connect (std::string path)
  {
    ShmdataAnyReader::ptr reader = std::make_shared<ShmdataAnyReader>();
    reader->set_data_type ("application/x-libloserialized-osc");
    reader->set_path (path);
    reader->set_callback(std::bind (&ShmdataToOsc::on_shmreader_data,
				    this,
				    std::placeholders::_1,
				    std::placeholders::_2,
				    std::placeholders::_3,
				    std::placeholders::_4,
				    std::placeholders::_5),
			 NULL);
    reader->start ();
    register_shmdata (reader);
    return true;
  }
 
  void
  ShmdataToOsc::on_shmreader_data (void *data,
				   int data_size,
				   unsigned long long timestamp,
				   const char *type_description, 
				   void *user_data)
  {
    const char *path = lo_get_path (data, data_size);
    lo_message msg = lo_message_deserialise (data, 
					     data_size, 
					     nullptr); //error code
    if (nullptr != msg )
      {
	std::unique_lock <std::mutex> lock (address_mutex_);
	//lo_message_pp (msg);
	if (nullptr != address_)
	  lo_send_message (address_, path, msg);
	lo_message_free (msg);
      }
  }

  bool 
  ShmdataToOsc::can_sink_caps (std::string caps)
  {
    return 0 == caps.find ("application/x-libloserialized-osc");
  }

}//end of ShmdataToOsc class


