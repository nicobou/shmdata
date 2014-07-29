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

#include "osc-to-shmdata.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(OscToShmdata,
				       "OSC message to shmdata",
				       "osc source", 
				       "OSCprop receives OSC messages and write to shmdata",
				       "LGPL",
				       "OSCshm",
				       "Nicolas Bouillot");

  OscToShmdata::OscToShmdata () :
    custom_props_ (new CustomPropertyHelper ()),
    port_ (1056),
    osc_thread_ (nullptr),
    port_spec_ (nullptr),
    start_ (std::chrono::system_clock::now()),
    shm_any_ (std::make_shared<ShmdataAnyWriter> ())
  {}
  
  bool
  OscToShmdata::init ()
  {
    init_startable (this);
    init_segment (this);
    port_spec_ = 
      custom_props_->make_int_property ("Port", 
					"OSC port to listen",
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

    return true;
  }

  OscToShmdata::~OscToShmdata ()
  {
    stop ();
  }

  void 
  OscToShmdata::set_port (const gint value, void *user_data)
  {
    OscToShmdata *context = static_cast <OscToShmdata *> (user_data);
    context->port_ = value;
  }
  
  gint 
  OscToShmdata::get_port (void *user_data)
  {
    OscToShmdata *context = static_cast <OscToShmdata *> (user_data);
    return context->port_;
  }

  bool 
  OscToShmdata::start ()
  {
    //creating a shmdata
    //ShmdataAnyWriter::ptr shm_any = std::make_shared<ShmdataAnyWriter> ();
    std::string shm_any_name = make_file_name ("osc");
    shm_any_->set_path (shm_any_name.c_str());
    g_message ("%s created a new shmdata any writer (%s)", 
	       get_nick_name ().c_str(), 
	       shm_any_name.c_str ());
    shm_any_->set_data_type ("application/x-libloserialized-osc");
    shm_any_->start ();
    register_shmdata (shm_any_);

    osc_thread_ = lo_server_thread_new (std::to_string(port_).c_str (), osc_error);
    if (nullptr == osc_thread_)
      return false;
    /* add method that will match any path and args */
    lo_server_thread_add_method (osc_thread_, nullptr, nullptr, osc_handler, this);
    lo_server_thread_start (osc_thread_);
    return true;
  }

  bool
  OscToShmdata::stop ()
  {
    unregister_shmdata (make_file_name ("osc"));
  {
      ShmdataAnyWriter::ptr temp = std::make_shared<ShmdataAnyWriter> ();
      std::swap (shm_any_,temp);
    }
    if (nullptr != osc_thread_)
      {
	lo_server_thread_free (osc_thread_);
	osc_thread_ = nullptr;
	return true;
      }
    return false;
  }

  /* catch any osc incoming messages. */
  int 
  OscToShmdata::osc_handler(const char *path, 
			     const char *types, 
			     lo_arg **argv,
			     int argc, 
			     lo_message m, 
			     void *user_data)
  {
    OscToShmdata *context = static_cast<OscToShmdata*>(user_data);
    lo_timetag timetag = lo_message_get_timestamp (m);
    //g_print ("timestamp %u %u", path, timetag.sec, timetag.frac);
    if (0 != timetag.sec)
      {
	//FIXME handle internal timetag
	//note: this is not implemented in osc-send
      }
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::chrono::duration<unsigned long long, std::nano> clock = now - context->start_;
    //g_print ("unknown osc path %s %llu", path, clock.count ());

    size_t size;
    void *buftmp = lo_message_serialise (m, path, nullptr, &size);
    context->shm_any_->push_data (buftmp,
     				  size,
     				  clock.count (),
     				  g_free,
     				  buftmp);
    return 0;
  }

  void 
  OscToShmdata::osc_error(int num, const char *msg, const char *path)
  {
    g_debug ("liblo server error %d in path %s: %s", num, path, msg);
  }
}//end of OscToShmdata class


