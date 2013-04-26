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

#include <string.h>
#include <glib.h> 
#include "switcher/webservices/soapcontrolProxy.h"
#include "switcher/webservices/control.nsmap"

//options
static gchar *server = NULL;
static gboolean createquiddity;
static gboolean deletequiddity;
static gboolean listclasses;
static gboolean classesdoc;
static gboolean classdoc;
static gboolean listquiddities;
static gboolean quidditydescr;
static gboolean quidditiesdescr;
static gboolean listprop;
static gboolean listpropbyclass;
static gboolean listmethods;
static gboolean listmethodsbyclass;
static gboolean setprop;
static gboolean getprop;
static gboolean invokemethod;
static gchar **remaining_args = NULL;

//FIXME the list-something should actually give lists, and json formated should be given without one-char option

static GOptionEntry entries[] =
  {
    { "server", 'S', 0, G_OPTION_ARG_STRING, &server, "server URI (default http://localhost:8080)", NULL },
    { "create-quiddity", 'C', 0, G_OPTION_ARG_NONE, &createquiddity, "create a quiddity instance (-C quiddity_class [optional nick name])", NULL },
    { "delete-quiddity", 'D', 0, G_OPTION_ARG_NONE, &deletequiddity, "delete a quiddity instance by its name", NULL },
    { "list-classes", 'c', 0, G_OPTION_ARG_NONE, &listclasses, "list quiddity classes", NULL },
    { "list-quiddities", 'e', 0, G_OPTION_ARG_NONE, &listquiddities, "list quiddity instances", NULL },
    { "list-props", 'p', 0, G_OPTION_ARG_NONE, &listprop, "list properties of a quiddity", NULL },
    { "list-props-by-class", 'P', 0, G_OPTION_ARG_NONE, &listpropbyclass, "list properties of a class", NULL },

    { "list-methods", 'm', 0, G_OPTION_ARG_NONE, &listmethods, "list methods of a quiddity", NULL },
    { "list-methods-by-class", 'M', 0, G_OPTION_ARG_NONE, &listmethodsbyclass, "list methods of a class", NULL },
    { "set-prop", 's', 0, G_OPTION_ARG_NONE, &setprop, "set property value (-s quiddity_name prop_name val)", NULL },
    { "get-prop", 'g', 0, G_OPTION_ARG_NONE, &getprop, "get property value (-g quiddity_name prop_name)", NULL },
    { "invoke-method", 'i', 0, G_OPTION_ARG_NONE, &invokemethod, "invoke method of a quiddity (-i quiddity_name method_name args...)", NULL },
    {G_OPTION_REMAINING, 0, 0, G_OPTION_ARG_STRING_ARRAY, &remaining_args, "remaining arguments", NULL},
    { "classes-doc", NULL, 0, G_OPTION_ARG_NONE, &classesdoc, "print classes documentation, JSON-formated", NULL },
    { "class-doc", NULL, 0, G_OPTION_ARG_NONE, &classdoc, "print class documentation, JSON-formated (--class-doc class_name)", NULL },
    { "quiddity-descr", NULL, 0, G_OPTION_ARG_NONE, &quidditydescr, "print quiddity documentation, JSON-formated (--class-doc class_name)", NULL },
    { "quiddities-descr", NULL, 0, G_OPTION_ARG_NONE, &quidditiesdescr, "print instanciated quiddities, JSON-formated", NULL },
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
  
  
  if (server == NULL)
    {
      server = "http://localhost:8080";
    }

  if (! (listclasses 
	 ^ classesdoc
	 ^ classdoc
	 ^ listquiddities 
	 ^ quidditydescr 
	 ^ quidditiesdescr 
	 ^ listprop 
	 ^ listpropbyclass 
	 ^ setprop 
	 ^ getprop 
	 ^ createquiddity 
	 ^ deletequiddity
	 ^ listmethods
	 ^ listmethodsbyclass
	 ^ invokemethod))
    {
      g_printerr ("I am very sorry for the inconvenience, but I am able to process only one command at a time. \n");
      exit (1);
    }

  controlProxy switcher_control(SOAP_IO_KEEPALIVE | SOAP_XML_INDENT);
  switcher_control.soap_endpoint = server;
  
  if (listclasses)
    {
      std::vector<std::string> resultlist;
      switcher_control.get_factory_capabilities(&resultlist);
      for(uint i = 0; i < resultlist.size(); i++)
	  std::cout << resultlist[i] << std::endl;
    }
  else if (classesdoc)
    {
      std::string result;
      switcher_control.get_classes_doc (&result);
      std::cout << result << std::endl;
    }
  else if (classdoc)
    {
      std::string resultlist;
      if (remaining_args[0] == NULL)
	{
	  g_printerr ("class name missing\n");
	  return false;
	}
      switcher_control.get_class_doc (remaining_args[0],&resultlist);
      std::cout << resultlist << std::endl;
    }
  else if (quidditydescr)
    {
      std::string resultlist;
      if (remaining_args[0] == NULL)
	{
	  g_printerr ("quiddity name missing\n");
	  return false;
	}
      switcher_control.get_quiddity_description (remaining_args[0],&resultlist);
      std::cout << resultlist << std::endl;
    }
  else if (quidditiesdescr)
    {
      std::string resultlist;
      switcher_control.get_quiddities_description (&resultlist);
      std::cout << resultlist << std::endl;
    }
  else if (listquiddities)
    {
      std::vector<std::string> resultlist;
      switcher_control.get_quiddity_names(&resultlist);
      for(uint i = 0; i < resultlist.size(); i++)
	{
	  std::cout << resultlist[i] << std::endl;
	}
    }
  else if (listprop)
    {
      std::string resultlist;
      if (remaining_args[0] == NULL)
	{
	  g_printerr ("quiddity name missing for listing properties\n");
	  return false;
	}
      if (remaining_args[1] == NULL)
	switcher_control.get_properties_description (remaining_args[0],
						     &resultlist);
      else
	switcher_control.get_property_description (remaining_args[0],
						   remaining_args[1],
						   &resultlist);
      std::cout << resultlist << std::endl;
    }
  else if (listpropbyclass)
    {
      std::string resultlist;
      if (remaining_args[0] == NULL)
	{
	  g_printerr ("class name missing for listing properties\n");
	  return false;
	}
      if (remaining_args[1] == NULL)
	switcher_control.get_properties_description_by_class (remaining_args[0],
							      &resultlist);
      else
	switcher_control.get_property_description_by_class (remaining_args[0],
							    remaining_args[1],
							    &resultlist);
      std::cout << resultlist << std::endl;
    }
  else if (setprop)
    {
      if (remaining_args[0] == NULL || remaining_args[1] == NULL || remaining_args[2] == NULL)
	{
	  g_printerr ("missing argument for set property\n");
	  return false;
	}
      //special since on
      switcher_control.send_set_property (remaining_args[0], remaining_args[1], remaining_args[2]);
      if (switcher_control.recv_set_property_empty_response())
	switcher_control.soap_print_fault(stderr);
      // connection should not be kept alive after the last call: be nice to the server and tell it that we close the connection after this call
      soap_clr_omode(&switcher_control, SOAP_IO_KEEPALIVE);
      switcher_control.soap_close_socket();
      return 0;
    }
  else if (getprop)
    {
      if (remaining_args[0] == NULL || remaining_args[1] == NULL)
	{
	  g_printerr ("missing argument for get property\n");
	  return false;
	}
      std::string val;
      switcher_control.get_property (remaining_args[0], remaining_args[1],&val);
      std::cout << val << std::endl;
    }
  else if (createquiddity)
    {
      if (remaining_args[0] == NULL )
	{
	  g_printerr ("missing class name for creating quiddity\n");
	  return false;
	}
      std::string name;
      if (remaining_args[1] == NULL)
	switcher_control.create_quiddity (remaining_args[0], &name);
      else
	switcher_control.create_named_quiddity (remaining_args[0], remaining_args[1], &name);
      std::cout << name << std::endl;
    }
  else if (deletequiddity)
    {
      if (remaining_args[0] == NULL )
	{
	  g_printerr ("missing quiddity name for deleting quiddity\n");
	  return false;
	}

      switcher_control.delete_quiddity (remaining_args[0]);
    }
  else if (listmethods)
    {
      if (remaining_args[0] == NULL )
	{
	  g_printerr ("missing quiddity name for list methods\n");
	  return false;
	}
      
      std::string resultlist;
      if (remaining_args[1] == NULL)
	switcher_control.get_methods_description(remaining_args[0], &resultlist);
      else
	switcher_control.get_method_description(remaining_args[0], remaining_args[1], &resultlist);
      std::cout << resultlist << std::endl;
    }
  else if (listmethodsbyclass)
    {
      if (remaining_args[0] == NULL )
	{
	  g_printerr ("missing quiddity name for list methods\n");
	  return false;
	}
      
      std::string resultlist;
      if (remaining_args[1] == NULL)
	switcher_control.get_methods_description_by_class (remaining_args[0], 
							   &resultlist);
      else
	switcher_control.get_method_description_by_class (remaining_args[0], 
							  remaining_args[1], 
							  &resultlist);
      std::cout << resultlist << std::endl;
    }
  else if (invokemethod)
    {
      if (remaining_args[0] == NULL || remaining_args[1] == NULL)
	{
	  g_printerr ("not enough argument for invoking a function\n");
	  return false;
	}
      std::vector<std::string> args;
      int i=2;
      while (remaining_args[i] != NULL)
      {
	  args.push_back (remaining_args[i]);
	  i++;
      }
      
      bool result;
      switcher_control.invoke_method (remaining_args[0],
				      remaining_args[1],
				      args,
				      &result);
      if (result) 
	g_print ("invocation returned true\n");
      else 
	g_print  ("invocation returned false\n");
    }

  if (switcher_control.error)
    switcher_control.soap_stream_fault(std::cerr);

    return 0;
}

