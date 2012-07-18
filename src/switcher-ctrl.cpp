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

#include <string.h>
#include <glib.h> 
#include "switcher/webservices/soapcontrolProxy.h"
#include "switcher/webservices/control.nsmap"

//options
static char server[] = "http://localhost:8080";
static gboolean listclasses;
static gboolean listentities;
static gboolean listprop;
static gboolean setprop;
static gboolean getprop;
static gboolean createentity;
static gboolean deleteentity;

static GOptionEntry entries[] =
  {
    { "server", 'S', 0, G_OPTION_ARG_STRING, &server, "server URI (default http://localhost:8080)", NULL },
    { "list-classes", 'c', 0, G_OPTION_ARG_NONE, &listclasses, "list entity classes", NULL },
    { "list-entities", 'e', 0, G_OPTION_ARG_NONE, &listentities, "list entity instances", NULL },
    { "list-prop", 'p', 0, G_OPTION_ARG_NONE, &listprop, "list properties of an entity", NULL },
    { "set-prop", 's', 0, G_OPTION_ARG_NONE, &setprop, "set property value (-s entity_name prop_name val)", NULL },
    { "get-prop", 'g', 0, G_OPTION_ARG_NONE, &getprop, "get property value (-g entity_name prop_name)", NULL },
    { "create-entity", 'C', 0, G_OPTION_ARG_NONE, &createentity, "create an entity instance (-C entity_class)", NULL },
    { "delete-prop", 'D', 0, G_OPTION_ARG_NONE, &deleteentity, "delete an entity instance by its name", NULL },
    { NULL }
};


int main(int argc, char **argv)
{ 

  //command line options
  GError *error = NULL;
  GOptionContext *context;
  context = g_option_context_new ("- switcher control via webservice");
  g_option_context_add_main_entries (context, entries, NULL);
  if (!g_option_context_parse (context, &argc, &argv, &error))
    {
      g_print ("option parsing failed: %s\n", error->message);
      exit (1);
    } 
  

  if (! (listclasses ^ listentities ^ listprop ^ setprop ^ getprop ^ createentity ^ deleteentity))
    {
      g_printerr ("I am very sorry for the inconvenience, but I am able to process only one command at a time. \n");
      exit (1);
    }

  //  controlProxy switcher_control;
  controlProxy switcher_control(SOAP_IO_KEEPALIVE | SOAP_XML_INDENT);
   switcher_control.soap_endpoint = server;
  
  if (listclasses)
    {
      std::vector<std::string> resultlist;
      switcher_control.get_factory_capabilities(&resultlist);
      for(uint i = 0; i < resultlist.size(); i++)
	{
	  std::cout << resultlist[i] << std::endl;
	}
    }
  else if (listentities)
    {
      std::vector<std::string> resultlist;
      switcher_control.get_entity_names(&resultlist);
      for(uint i = 0; i < resultlist.size(); i++)
	{
	  std::cout << resultlist[i] << std::endl;
	}
    }
  else if (listprop)
    {
      std::vector<std::string> resultlist;
      switcher_control.get_property_names(argv[argc-1],&resultlist);
      for(uint i = 0; i < resultlist.size(); i++)
	{
	  std::cout << resultlist[i] << std::endl;
	}
    }
  else if (setprop)
    {
      //special since on
      switcher_control.send_set_property (argv[argc-3], argv[argc-2], argv[argc-1]);
      if (switcher_control.recv_set_property_empty_response())
	switcher_control.soap_print_fault(stderr);
      // connection should not be kept alive after the last call: be nice to the server and tell it that we close the connection after this call
      soap_clr_omode(&switcher_control, SOAP_IO_KEEPALIVE);
      switcher_control.soap_close_socket();
      return 0;
    }
  else if (getprop)
    {
      std::string val;
      switcher_control.get_property (argv[argc-2], argv[argc-1],&val);
      std::cout << val << std::endl;
    }
  else if (createentity)
    {
      std::string name;
      switcher_control.create_entity (argv[argc-1], &name);
      std::cout << name << std::endl;
    }
  else if (deleteentity)
    {
      switcher_control.delete_entity (argv[argc-1]);
    }
  
  if (switcher_control.error)
    switcher_control.soap_stream_fault(std::cerr);

    return 0;
}

