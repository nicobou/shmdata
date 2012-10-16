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

#include "switcher/udpsink.h"

namespace switcher
{

  UDPSink::UDPSink (QuiddityLifeManager::ptr life_manager)
  {
    life_manager_ = life_manager;
    make_udpsink ();
  }

  
  UDPSink::UDPSink ()
  {
    make_udpsink ();
  }

  void 
  UDPSink::make_udpsink ()
  {
    udpsink_ = gst_element_factory_make ("multiudpsink",NULL);
    
    //set the name before registering properties
    set_name (gst_element_get_name (udpsink_));
    g_object_set (G_OBJECT (udpsink_), "sync", FALSE, NULL);
    
    register_property (G_OBJECT (udpsink_),"blocksize","");
    register_property (G_OBJECT (udpsink_),"bytes-served","");
    register_property (G_OBJECT (udpsink_),"clients","");
    register_property (G_OBJECT (udpsink_),"ttl","");
    register_property (G_OBJECT (udpsink_),"ttl-mc","");
    register_property (G_OBJECT (udpsink_),"loop","");
    
    // g_signal_connect (G_OBJECT (udpsink_), "client-added",  
    // 		      (GCallback)  on_client_added, (gpointer) this);
    // g_signal_connect (G_OBJECT (udpsink_), "client-removed",  
    // 		      (GCallback)  on_client_removed, (gpointer) this);
    
     //registering add_client
     std::vector<GType> add_client_arg_types;
     add_client_arg_types.push_back (G_TYPE_STRING);
     add_client_arg_types.push_back (G_TYPE_INT);
     register_method("add_client",
     		    (void *)&add_client_wrapped, 
     		    add_client_arg_types,
     		    (gpointer)this);
     std::vector<std::pair<std::string,std::string> > add_client_arg_desc;
     std::pair<std::string,std::string> add_client_host;
     add_client_host.first = "host";
     add_client_host.second = "the hostname/IP address of the client to add";
     add_client_arg_desc.push_back (add_client_host); 
     std::pair<std::string,std::string> add_client_port;
     add_client_port.first = "port";
     add_client_port.second = "the port of the client to add";
     add_client_arg_desc.push_back (add_client_port); 
    
     if (!set_method_description ("add_client", 
     				 "add a client with destination host and port to the list of clients", 
     				 add_client_arg_desc))
       g_printerr ("UDP sink: cannot set method description for \"add_client\"\n");

     //registering remove_client
     std::vector<GType> remove_client_arg_types;
     remove_client_arg_types.push_back (G_TYPE_STRING);
     remove_client_arg_types.push_back (G_TYPE_INT);
     register_method("remove_client",
     		    (void *)&remove_client_wrapped, 
     		    remove_client_arg_types,
     		    (gpointer)this);
     std::vector<std::pair<std::string,std::string> > remove_client_arg_desc;
     std::pair<std::string,std::string> remove_client_host;
     remove_client_host.first = "host";
     remove_client_host.second = "the hostname/IP address of the client to remove";
     remove_client_arg_desc.push_back (remove_client_host); 
     std::pair<std::string,std::string> remove_client_port;
     remove_client_port.first = "port";
     remove_client_port.second = "the port of the client to remove";
     remove_client_arg_desc.push_back (remove_client_port); 
    
     if (!set_method_description ("remove_client", 
     				 "remove a client with destination host and port to the list of clients", 
     				 remove_client_arg_desc))
       g_printerr ("UDP sink: cannot set method description for \"remove_client\"\n");

     //registering clear
     std::vector<GType> clear_arg_types;
     register_method("clear",
     		    (void *)&clear_wrapped, 
     		    clear_arg_types,
     		    (gpointer)this);
     std::vector<std::pair<std::string,std::string> > clear_arg_desc;
     if (!set_method_description ("clear", 
				  "remove a client with destination host and port to the list of clients", 
				  clear_arg_desc))
       g_printerr ("UDP sink: cannot set method description for \"clear\"\n");
     

    set_sink_element (udpsink_);
  }
  

  QuiddityDocumentation UDPSink::doc_ ("udp sink", "udpsink",
				       "send data stream with udp");

  QuiddityDocumentation 
  UDPSink::get_documentation ()
  {
    return doc_;
  }
  
  void 
  UDPSink::on_client_added (GstElement *multiudpsink, gchar *host, gint port, gpointer user_data)
  {
    //UDPSink *context = static_cast<UDPSink *>(user_data);
    //g_print ("UDPSink::on_client_added\n");
  }

  void 
  UDPSink::on_client_removed (GstElement *multiudpsink, gchar *host, gint port, gpointer user_data)
  {
    //UDPSink *context = static_cast<UDPSink *>(user_data);
    //g_print ("UDPSink::on_client_removed\n");
  }

  gboolean
  UDPSink::remove_client_wrapped (gpointer host, gint port, gpointer user_data)
  {
    //std::string connector = static_cast<std::string>(connector_name);
    UDPSink *context = static_cast<UDPSink*>(user_data);
    
    if (context->remove_client ((char *)host, port))
      return TRUE;
    else
      return FALSE;
  }

  bool 
  UDPSink::remove_client (gchar *host, gint port)
  {
    g_signal_emit_by_name (udpsink_, "remove", host, port, NULL);
    return true;
  }

  gboolean
  UDPSink::add_client_wrapped (gpointer host, gint port, gpointer user_data)
  {
    //std::string connector = static_cast<std::string>(connector_name);
    UDPSink *context = static_cast<UDPSink*>(user_data);
       
    if (context->add_client ((char *)host, port))
      return TRUE;
    else
      return FALSE;
  }

  bool 
  UDPSink::add_client (gchar *host, gint port)
  {
    g_signal_emit_by_name (udpsink_, "add", host, port, NULL);
    return true;
  }

 gboolean
  UDPSink::clear_wrapped (gpointer user_data)
  {
    //std::string connector = static_cast<std::string>(connector_name);
    UDPSink *context = static_cast<UDPSink*>(user_data);
       
    if (context->clear_clients ())
      return TRUE;
    else
      return FALSE;
  }

  bool 
  UDPSink::clear_clients ()
  {
    g_signal_emit_by_name (udpsink_, "clear", NULL);
    return true;
  }

}
