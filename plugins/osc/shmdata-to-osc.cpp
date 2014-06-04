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
    shmdata_path_ (),
    port_spec_ (NULL),
    host_spec_ (NULL),
    shmdata_path_spec_ (NULL),
    address_ (NULL),
    reader_ (shmdata_any_reader_init ()),
    address_mutex_ ()
  {}
  
  bool
  ShmdataToOsc::init_segment ()
  {
    init_startable (this);
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

    shmdata_path_spec_ = 
      custom_props_->make_string_property ("shmdata-path", 
					   "path of the shmdata to connect with",
					   shmdata_path_.c_str (),
					   (GParamFlags) G_PARAM_READWRITE,
					   set_shmdata_path,
					   get_shmdata_path,
					   this);
    
    install_property_by_pspec (custom_props_->get_gobject (), 
				shmdata_path_spec_, 
				"shmdata-path",
				"Shmdata Path");

    shmdata_any_reader_set_debug (reader_, SHMDATA_ENABLE_DEBUG);
    shmdata_any_reader_set_on_data_handler (reader_, 
					    &on_shmreader_data,
					    this);
    shmdata_any_reader_set_data_type (reader_, "application/x-json-osc");

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
    if (NULL == address_)
      return false;
    return true;
  }

  bool
  ShmdataToOsc::stop ()
  {
    if (NULL != address_)
      {
	std::unique_lock <std::mutex> lock (address_mutex_);
	lo_address_free (address_);
	address_ = NULL;
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


  void 
  ShmdataToOsc::set_shmdata_path (const gchar * value, void *user_data)
  {
    ShmdataToOsc *context = static_cast <ShmdataToOsc *> (user_data);
    context->shmdata_path_ = value;
    shmdata_any_reader_start (context->reader_, context->shmdata_path_.c_str ());
  }
 
  const gchar *
  ShmdataToOsc::get_shmdata_path (void *user_data)
  {
    ShmdataToOsc *context = static_cast <ShmdataToOsc *> (user_data);
    return context->shmdata_path_.c_str ();
  }

  void
  ShmdataToOsc::on_shmreader_data (shmdata_any_reader_t */*reader*/,
				   void *shmbuf,
				   void *data,
				   int data_size,
				   unsigned long long timestamp,
				   const char *type_description, 
				   void *user_data)
  {
    ShmdataToOsc *context = static_cast <ShmdataToOsc *> (user_data);
    const char *path = lo_get_path (data, data_size);
    lo_message msg = lo_message_deserialise (data, 
					     data_size, 
					     NULL); //error code
    if (NULL != msg)
      {
	//lo_message_pp (msg);
	std::unique_lock <std::mutex> lock (context->address_mutex_);
	lo_send_message (context->address_, path, msg);
	lo_message_free (msg);
      }
    shmdata_any_reader_free (shmbuf);
  }

}//end of ShmdataToOsc class


