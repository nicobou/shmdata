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
#include "osc-to-shmdata.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(OscToShmdata,
				       "OSC message to property",
				       "osc source", 
				       "OSCprop reveives OSC messages and updates associated property",
				       "LGPL",
				       "OSCprop",
				       "Nicolas Bouillot");

  OscToShmdata::OscToShmdata () :
    custom_props_ (new CustomPropertyHelper ()),
    port_ (1056),
    osc_thread_ (NULL),
    port_spec_ (NULL),
    start_ (std::chrono::system_clock::now()),
    shm_any_ (std::make_shared<ShmdataAnyWriter> ())
  {}
  
  bool
  OscToShmdata::init_segment ()
  {
    init_startable (this);
    port_spec_ = 
      custom_props_->make_int_property ("Port", 
					"osc port to listen",
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

    //creating a shmdata
    //ShmdataAnyWriter::ptr shm_any = std::make_shared<ShmdataAnyWriter> ();
    std::string shm_any_name = make_file_name ("osc");
    shm_any_->set_path (shm_any_name.c_str());
    g_message ("%s created a new shmdata any writer (%s)", 
	       get_nick_name ().c_str(), 
	       shm_any_name.c_str ());
    shm_any_->set_data_type ("application/x-json-osc");
    shm_any_->start ();
    register_shmdata_any_writer (shm_any_);

    return true;
  }

  OscToShmdata::~OscToShmdata ()
  {
    stop ();
  }


  //floor
  gchar *
  OscToShmdata::string_float_to_string_int (const gchar *string_float)
  {
    gchar *res;
    gchar **split= g_strsplit (string_float,
			       ".",
			       2);
    res = g_strdup (split[0]);
    g_strfreev(split);
    return res;
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
    osc_thread_ = lo_server_thread_new (std::to_string(port_).c_str (), osc_error);
    if (NULL == osc_thread_)
      return false;
    /* add method that will match any path and args */
    lo_server_thread_add_method (osc_thread_, NULL, NULL, osc_handler, this);
    lo_server_thread_start (osc_thread_);
    return true;
  }

  bool
  OscToShmdata::stop ()
  {
    if (NULL != osc_thread_)
      {
	lo_server_thread_free (osc_thread_);
	osc_thread_ = NULL;
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
    std::string buf = osc_to_json (path, types, argv, argc);
    gchar *buftmp = g_strdup (buf.c_str ());
    context->shm_any_->push_data (buftmp,
				  sizeof(char) * (buf.size () + 1),
				  clock.count (),
				  g_free,
				  buftmp);
    return 0;
  }

  
  std::string
  OscToShmdata::osc_to_json (const char *path, 
			      const char *types, 
			      lo_arg **argv,
			      int argc)
  {
    JSONBuilder::ptr json_builder = std::make_shared<JSONBuilder> ();
    json_builder->begin_object ();
    json_builder->add_string_member ("path", path);
    json_builder->set_member_name ("values");
    json_builder->begin_array ();
    // verbose:
    // for (int i = 1; i < argc; i++)
    //   {
    // 	json_builder->begin_object ();
    // 	const char * value_type = std::string  (1, types[i]).c_str ();
    // 	json_builder->add_string_member ("type", value_type);
    // 	gchar *value = string_from_osc_arg (types[i], argv[i]);
    // 	json_builder->add_string_member ("value", value);
    // 	g_free (value);
    // 	json_builder->end_object ();
    //   }
    json_builder->add_string_value (types + 1); //first char is 's', probably string for the path
    for (int i = 1; i < argc; i++)
      {
    	gchar *value = string_from_osc_arg (types[i], argv[i]);
    	json_builder->add_string_value (value);
    	g_free (value);
      }
    json_builder->end_array ();
    json_builder->end_object ();
    return json_builder->get_string (false);
  }
 
  gchar *
  OscToShmdata::string_from_osc_arg (char type, lo_arg *data)
  {
    gchar *res = NULL;
    gchar *tmp = NULL; 
    switch (type) {
    case LO_INT32:
      res = g_strdup_printf ("%d", data->i);
      break;
       
    case LO_FLOAT:
      tmp = g_strdup_printf ("%f", data->f);
      if (g_str_has_suffix (tmp, ".000000")) // for pd
	{
	  res = string_float_to_string_int (tmp);
	  g_free (tmp);
	}
      else
	res = tmp;
      break;

    case LO_STRING:
      res = g_strdup_printf ("%s", (char *)data);
      break;

    case LO_BLOB:
      g_debug ("LO_BLOB not handled");
      break;

    case LO_INT64:
      res = g_strdup_printf ("%lld", (long long int)data->i);
      break;
    
    case LO_TIMETAG:
      g_debug ("LO_BLOB not handled");
      break;
    
    case LO_DOUBLE:
      tmp = g_strdup_printf ("%f", data->f);
      if (g_str_has_suffix (tmp, ".000000")) // for pd
	{
	  res = string_float_to_string_int (tmp);
	  g_free (tmp);
	}
      else
	res = tmp;
      break;
    
    case LO_SYMBOL:
      res = g_strdup_printf ("'%s", (char *)data);
      break;

    case LO_CHAR:
      res = g_strdup_printf ("'%c'", (char)data->c);
      break;

    case LO_MIDI:
      // res = g_strdup_printf ("MIDI [");
      // for (i=0; i<4; i++) {
      //     res = g_strdup_printf ("0x%02x", *((uint8_t *)(data) + i));
      //     if (i+1 < 4) res = g_strdup_printf (" ");
      // }
      // res = g_strdup_printf ("]");
      g_debug ("LO_MIDI not handled");
      break;
    case LO_TRUE:
      res = g_strdup_printf ("true");
      break;

    case LO_FALSE:
      res = g_strdup_printf ("false");
      break;

    case LO_NIL:
      res = g_strdup_printf ("Nil");
      break;

    case LO_INFINITUM:
      res = g_strdup_printf ("Infinitum");
      break;

    default:
      g_warning ("unhandled liblo type: %c\n", type);
      break;
    }
    return res;
  }

  void 
  OscToShmdata::osc_error(int num, const char *msg, const char *path)
  {
    g_debug ("liblo server error %d in path %s: %s", num, path, msg);
  }
}//end of OscToShmdata class


