/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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

#include "osc-to-property.h"

//#include <stdio.h>
//#include <stdlib.h>
//#include <unistd.h>
//#include <utility> //std::make_pair (,)

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(OscToProperty,
				       "OSC message to property",
				       "network converter", 
				       "OSCprop reveives OSC messages and updates associated property",
				       "LGPL",
				       "OSCprop",
				       "Nicolas Bouillot");
    
  OscToProperty::OscToProperty () :
    custom_props_ (new CustomPropertyHelper ()),
    port_ (),
    osc_thread_ (NULL)
  {}

  bool
  OscToProperty::init ()
  {
    install_method ("Set Port",
		    "set_port", 
		    "set the port used by the osc server and start listening messages", 
		    "success or fail",
		    Method::make_arg_description ("Port",
						  "port",
						  "the port to bind",
						  NULL),
		    (Method::method_ptr) &set_port_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
     		    this);

    return true;
  }

  OscToProperty::~OscToProperty ()
  {
    stop ();
  }


  //floor
  gchar *
  OscToProperty::string_float_to_string_int (const gchar *string_float)
  {
    gchar *res;
    gchar **split= g_strsplit (string_float,
			       ".",
			       2);
    res = g_strdup (split[0]);
    g_strfreev(split);
    return res;
  }

  gboolean
  OscToProperty::set_port_wrapped (gpointer port, gpointer user_data)
  {
    OscToProperty *context = static_cast<OscToProperty*>(user_data);
    context->set_port ((char *)port);
    return TRUE;
  }

  void 
  OscToProperty::set_port (std::string port)
  {
    stop ();
    port_ = port;
    start ();
  }

  void 
  OscToProperty::start ()
  {
    osc_thread_ = lo_server_thread_new(port_.c_str (), osc_error);
    /* add method that will match any path and args */
    lo_server_thread_add_method(osc_thread_, NULL, NULL, osc_handler, this);
    lo_server_thread_start(osc_thread_);
  }

  void
  OscToProperty::stop ()
  {
    if (osc_thread_ != NULL)
      lo_server_thread_free(osc_thread_);
    osc_thread_ = NULL;
  }


  /* catch any osc incoming messages. */
  int 
  OscToProperty::osc_handler(const char *path, 
			     const char */*types*/, 
			     lo_arg **/*argv*/,
			     int /*argc*/, 
			     void */*data*/, 
			     void */*user_data*/)
  {
    //OscToProperty *context = static_cast<OscToProperty*>(user_data);
    g_debug ("unknown osc path %s", path);
    return 0;
  }
 
  gchar *
  OscToProperty::string_from_osc_arg (char type, lo_arg *data)
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
  OscToProperty::osc_error(int num, const char *msg, const char *path)
  {
    g_debug ("liblo server error %d in path %s: %s", num, path, msg);
  }
}//end of OscToProperty class


