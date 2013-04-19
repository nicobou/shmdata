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

#include "switcher/osc-ctrl-server.h"
#include <ctime>    // For time()
#include <cstdlib>  // For srand() and rand()

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <lo/lo.h>

namespace switcher
{
  QuiddityDocumentation OscCtrlServer::doc_ ("control", "OSCctl",
					     "OSCcontrolServer allows for managing switcher through OSC");
    
  bool
  OscCtrlServer::init ()
  {
    port_ = 5446;
    srand(time(0));
    set_name (g_strdup_printf ("oscctrlserver%d",rand() % 1024));
   
    //registering set_port
    register_method("set_port",
     		    (void *)&set_port_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
     		    (gpointer)this);
    set_method_description ("set_port", 
			    "set the port used by the osc server and start listening messages", 
			    Method::make_arg_description ("port",
							  "the port to bind",
							  NULL));
    return true;
  }

  OscCtrlServer::~OscCtrlServer ()
  {
    stop ();
  }

  std::shared_ptr<QuiddityManager>
  OscCtrlServer::get_quiddity_manager ()
  {
    return manager_.lock ();
  }

  QuiddityDocumentation 
  OscCtrlServer::get_documentation ()
  {
    return doc_;
  }
  

  gboolean
  OscCtrlServer::set_port_wrapped (gpointer port, gpointer user_data)
  {
    OscCtrlServer *context = static_cast<OscCtrlServer*>(user_data);
    context->set_port ((char *)port);
    return TRUE;
  }

  void 
  OscCtrlServer::set_port (std::string port)
  {
    stop ();
    port_ = port;
    start ();
  }

  void 
  OscCtrlServer::start ()
  {
    osc_thread_ = lo_server_thread_new(port_.c_str (), osc_error);
        /* add method that will match any path and args */
    lo_server_thread_add_method(osc_thread_, NULL, NULL, osc_handler, this);
    lo_server_thread_start(osc_thread_);
  }

  void
  OscCtrlServer::stop ()
  {
    lo_server_thread_free(osc_thread_);
  }


  /* catch any osc incoming messages. */
  int 
  OscCtrlServer::osc_handler(const char *path, const char *types, lo_arg **argv,
			     int argc, void *data, void *user_data)
  {
    OscCtrlServer *context = static_cast<OscCtrlServer*>(user_data);
    std::shared_ptr<QuiddityManager> manager = context->get_quiddity_manager ();

    //create 
    if (g_str_has_prefix (path, "/c") || g_str_has_prefix (path, "/C"))
      {
	if (argc == 1)
	  {
	    gchar *class_name = string_from_osc_arg (types[0], argv[0]);
	    manager->create (class_name);
	    g_free (class_name); 
	  }
	else if  (argc == 2)
	  {
	    gchar *class_name = string_from_osc_arg (types[0], argv[0]);
	    gchar *quid_name = string_from_osc_arg (types[1], argv[1]);
	    manager->create (class_name, quid_name);
	    g_free (class_name); 
	    g_free (quid_name); 
	  }
	else
	    g_warning ("OSCctl: wrong arg number for create");
	return 0;
      }

    //remove 
    if (g_str_has_prefix (path, "/r") || g_str_has_prefix (path, "/R"))
      {
	if (argc == 1)
	  {
	    gchar *quid_name = string_from_osc_arg (types[0], argv[0]);
	    manager->remove (quid_name);
	    g_free (quid_name); 
	  }
	else
	  g_warning ("OSCctl: wrong arg number for remove");
	return 0;
      }
    
    //set_property 
    if (g_str_has_prefix (path, "/s") || g_str_has_prefix (path, "/S"))
      {
	if (argc == 3)
	  {
	    gchar *quid_name = string_from_osc_arg (types[0], argv[0]);
	    gchar *prop_name = string_from_osc_arg (types[1], argv[1]);
	    gchar *value = string_from_osc_arg (types[2], argv[2]);
	    manager->set_property (quid_name, prop_name, value);
	    g_free (quid_name); 
	    g_free (prop_name); 
	    g_free (value); 
	  }
	else
	  g_warning ("OSCctl: wrong arg number for set_property");
	return 0;
      }

    //invoke 
    if (g_str_has_prefix (path, "/i") || g_str_has_prefix (path, "/I"))
      {
	if (argc > 2)
	  {
	    gchar *quid_name = string_from_osc_arg (types[0], argv[0]);
	    gchar *method_name = string_from_osc_arg (types[1], argv[1]);
	    int i;
	    std::vector<std::string> args;
	    for (i = 2; i < argc; i++)
	      {
		gchar *val = string_from_osc_arg (types[i], argv[i]);
		args.push_back (val);
		g_free (val);
	      }
	    manager->invoke (quid_name, method_name, args);
	    g_free (quid_name); 
	    g_free (method_name); 
	  }
	else
	  g_warning ("OSCctl: wrong arg number for invoke");
	return 0;
      }
    
    return 0;
  }
 
  gchar *
  OscCtrlServer::string_from_osc_arg (char type, lo_arg *data)
  {
    //lo_arg_host_endian ((lo_type) type, data);
    char *res;// = g_strdup_printf ("videotestsrc");

     switch (type) {
     case LO_INT32:
       res = g_strdup_printf ("%d", data->i);
       break;
       
     case LO_FLOAT:
 	res = g_strdup_printf ("%f", data->f);
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
 	res = g_strdup_printf ("%f", data->f);
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
       g_debug ("LO_BLOB not handled");
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
  OscCtrlServer::osc_error(int num, const char *msg, const char *path)
  {
    g_debug ("liblo server error %d in path %s: %s", num, path, msg);
  }
}//end of OscCtrlServer class


