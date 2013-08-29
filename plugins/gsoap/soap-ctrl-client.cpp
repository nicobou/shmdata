/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher-gsoap.
 *
 * switcher-gsoap is free software: you can redistribute it and/or modify
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

#include "soap-ctrl-client.h"
#include <ctime>    // For time()
#include <cstdlib>  // For srand() and rand()

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(SoapCtrlClient,
				       "Switcher Web Client (SOAP)",
				       "control client",
				       "controling a switcher instance through SOAP webservices",
				       "GPL",
				       "SOAPcontrolClient",
				       "Nicolas Bouillot");

  bool
  SoapCtrlClient::init()
  {
    
    srand(time(0));
    set_name (g_strdup_printf ("ctrlclient%d",rand() % 1024));

    switcher_control_ = new controlProxy (SOAP_IO_KEEPALIVE | SOAP_XML_INDENT);
    url_ = NULL;
    switcher_control_->soap_endpoint = url_;


    publish_method ("Set Remote Switcher",
		    "set_remote_url", 
		    "set remote url to control (for instance http://localhost:8080)", 
		    "success or fail",
		    Method::make_arg_description ("URL",
						  "url",
						  "SOAP url",
						  NULL),
		    (Method::method_ptr) &set_remote_url_wrapped, 
		    G_TYPE_BOOLEAN,
     		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
     		    this);


    publish_method ("Create",
		    "create", 
		    "create a quiddity", 
		    "success or fail",
		    Method::make_arg_description ("Quiddity Class Name",
						  "class_name",
						  "the class of the quiddity to create",
						  "Quiddity Name",
						  "quiddity_name",
						  "the name to give",
						  NULL),
		    (Method::method_ptr) &create_wrapped, 
		    G_TYPE_BOOLEAN,
     		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, NULL),
     		    this);

  
    publish_method ("Remove",
		    "remove", 
		    "remove a quiddity", 
		    "success or fail",
		    Method::make_arg_description ("Quiddity Name",
						  "quiddity_name",
						  "name of quiddity to remove",
						  NULL),
		    (Method::method_ptr) &remove_wrapped, 
		    G_TYPE_BOOLEAN,
     		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
     		    this);
 
     
    publish_method ("Set Property",
		    "set_property", 
		    "set a quiddity property", 
		    "success or fail",
		    Method::make_arg_description ("Quiddity Name",
						  "quiddity_name",
						  "name of quiddity to remove",
						  "Property Name",
						  "property_name",
						  "name of the property",
						  "Property Value",
						  "property_value",
						  "value to set",
						  NULL),
		    (Method::method_ptr) &set_property_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING, NULL),
		    this);


    publish_method ("Invoke 1",
		    "invoke1", 
		    "invoke a method with one argument", 
		    "success or fail",
		    Method::make_arg_description ("Quiddity Name ",
						  "quiddity_name",
						  "name of quiddity to remove",
						  "Method Name",
						  "method_name",
						  "name of the method",
						  "First Argument",
						  "arg1",
						  "first argument",
						  NULL),
		    (Method::method_ptr) &invoke1_wrapped, 
		    G_TYPE_BOOLEAN,
      		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING, NULL),
      		    this); 


    publish_method ("Invoke 2",
		    "invoke2", 
		    "invoke a method with two arguments", 
		    "success or fail",
		    Method::make_arg_description ("quiddity Name",
						  "quiddity_name",
						  "name of quiddity to remove",
						  "Method Name",
						  "method_name",
						  "name of the method",
						  "First Argument",
						  "arg1",
						  "first argument",
						  "Second Argument",
						  "arg2",
						  "second argument",
						  NULL),
		    (Method::method_ptr) &invoke2_wrapped, 
		    G_TYPE_BOOLEAN,
     		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, 
     						       G_TYPE_STRING, G_TYPE_STRING, NULL),
     		    this);


    publish_method ("Invoke 3",
		    "invoke3", 
		    "invoke a method with three arguments", 
		    "success or fail",
		    Method::make_arg_description ("quiddity Name",
						  "quiddity_name",
						  "name of quiddity to remove",
						  "Method Name",
						  "method_name",
						  "name of the method",
						  "First Argument",
						  "arg1",
						  "first argument",
						  "Second Argument",
						  "arg2",
						  "second argument",
						  "Third Argument",
						  "arg3",
						  "third argument",
						  NULL),
		    (Method::method_ptr) &invoke3_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, 
						       G_TYPE_STRING, G_TYPE_STRING, 
						       G_TYPE_STRING, NULL),
		    this);

    publish_method ("Invoke 4",
		    "invoke4", 
		    "invoke a method with four arguments", 
		    "success or fail",
		    Method::make_arg_description ("quiddity Name",
						  "quiddity_name",
						  "name of quiddity to remove",
						  "Method Name",
						  "method_name",
						  "name of the method",
						  "First Argument",
						  "arg1",
						  "first argument",
						  "Second Argument",
						  "arg2",
						  "second argument",
						  "Third Argument",
						  "arg3",
						  "third argument",
						  "Fourth Argument",
						  "arg4",
						  "fourth argument",
						  NULL),
		    (Method::method_ptr) &invoke4_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, 
						       G_TYPE_STRING, G_TYPE_STRING, 
						       G_TYPE_STRING, G_TYPE_STRING, NULL),
		    this);
    return true;
  }

  SoapCtrlClient::~SoapCtrlClient()
  {
    if (url_ != NULL)
      g_free (url_);
    if (switcher_control_ != NULL)
      delete switcher_control_;
  }

  gboolean
  SoapCtrlClient::set_remote_url_wrapped (gpointer url,
					  gpointer user_data)
  {
    SoapCtrlClient *context = static_cast<SoapCtrlClient *> (user_data);
    if (context->url_ != NULL)
      g_free (context->url_);

    context->url_ = g_strdup ((char *)url);
    context->switcher_control_->soap_endpoint = context->url_;
    
    std::vector<std::string> resultlist;
    context->switcher_control_->get_quiddity_names(&resultlist);
    
    if (context->switcher_control_->error)
      {
	g_warning ("SoapCtrlClient::set_remote_url, url not valid or not responding");
	g_free (context->url_);
	context->url_ = NULL;
	return FALSE;
      }
    return TRUE;
  }


  gboolean
  SoapCtrlClient::create_wrapped (gpointer class_name,
				  gpointer quiddity_name,
				  gpointer user_data)
  {
    SoapCtrlClient *context = static_cast<SoapCtrlClient *> (user_data);
    if (context->url_ == NULL)
      return FALSE;
    std::string name;
    context->switcher_control_->create_named_quiddity ((const char *)class_name, 
						      (const char *)quiddity_name,
						      &name);
    if (g_strcmp0 ((gchar *)quiddity_name, name.c_str ()) != 0)
      {
	context->switcher_control_->delete_quiddity (name.c_str ());
	return FALSE;
      }
    return TRUE;
  }

  gboolean
  SoapCtrlClient::remove_wrapped (gpointer quiddity_name,
				  gpointer user_data)
  {
    SoapCtrlClient *context = static_cast<SoapCtrlClient *> (user_data);
    if (context->url_ == NULL)
      return FALSE;
    context->switcher_control_->delete_quiddity ((gchar *)quiddity_name);
    if (context->switcher_control_->error)
      return FALSE;
    return TRUE;
  }


  gboolean
  SoapCtrlClient::set_property_wrapped (gpointer quiddity_name,
					gpointer property_name,
					gpointer value,
					gpointer user_data)
  {
    SoapCtrlClient *context = static_cast<SoapCtrlClient *> (user_data);
    if (context->url_ == NULL)
      return FALSE;
    context->switcher_control_->send_set_property ((gchar *)quiddity_name, 
						   (gchar *)property_name, 
						   (gchar *)value);
    if (context->switcher_control_->recv_set_property_empty_response())
      return FALSE;//context->switcher_control_->soap_print_fault(stderr);

    // connection should not be kept alive after the last call: be nice to the server and tell it that we close the connection after this call
    soap_clr_omode((context->switcher_control_), SOAP_IO_KEEPALIVE);
    context->switcher_control_->soap_close_socket();
    return TRUE;
  }

  gboolean
  SoapCtrlClient::invoke1_wrapped (gpointer quiddity_name,
				   gpointer method_name,
				   gpointer arg1,
				   gpointer user_data)
  {
    SoapCtrlClient *context = static_cast<SoapCtrlClient *> (user_data);
    if (context->url_ == NULL)
      return FALSE;

    std::vector<std::string> args;
    args.push_back ((char *) arg1);
    std::string result;
    context->switcher_control_->invoke_method ((gchar *)quiddity_name,
					       (gchar *)method_name,
					       args,
					       &result);
    return TRUE;
  }

  gboolean
  SoapCtrlClient::invoke2_wrapped (gpointer quiddity_name,
				   gpointer method_name,
				   gpointer arg1,
				   gpointer arg2,
				   gpointer user_data)
  {
    SoapCtrlClient *context = static_cast<SoapCtrlClient *> (user_data);
    if (context->url_ == NULL)
      return FALSE;
    
    std::vector<std::string> args;
    args.push_back ((char *) arg1);
    args.push_back ((char *) arg2);
    std::string result;
    context->switcher_control_->invoke_method ((gchar *)quiddity_name,
					       (gchar *)method_name,
					       args,
					       &result);
    return TRUE;
  }

  gboolean
  SoapCtrlClient::invoke3_wrapped (gpointer quiddity_name,
				   gpointer method_name,
				   gpointer arg1,
				   gpointer arg2,
				   gpointer arg3,
				   gpointer user_data)
  {
    SoapCtrlClient *context = static_cast<SoapCtrlClient *> (user_data);
    if (context->url_ == NULL)
      return FALSE;
    
    std::vector<std::string> args;
    args.push_back ((char *) arg1);
    args.push_back ((char *) arg2);
    args.push_back ((char *) arg3);
    std::string result;
    context->switcher_control_->invoke_method ((gchar *)quiddity_name,
					       (gchar *)method_name,
					       args,
					       &result);
    return TRUE;
  }

  gboolean
  SoapCtrlClient::invoke4_wrapped (gpointer quiddity_name,
				   gpointer method_name,
				   gpointer arg1,
				   gpointer arg2,
				   gpointer arg3,
				   gpointer arg4,
				   gpointer user_data)
  {
    SoapCtrlClient *context = static_cast<SoapCtrlClient *> (user_data);
    if (context->url_ == NULL)
      return FALSE;
    
    std::vector<std::string> args;
    args.push_back ((char *) arg1);
    args.push_back ((char *) arg2);
    args.push_back ((char *) arg3);
    args.push_back ((char *) arg4);
    std::string result;
    context->switcher_control_->invoke_method ((gchar *)quiddity_name,
					       (gchar *)method_name,
					       args,
					       &result);
    return TRUE;
  }
}
