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
    udpsink_bin_ = gst_element_factory_make ("bin",NULL);
    typefind_ =  gst_element_factory_make ("typefind",NULL);
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

    register_property (G_OBJECT (typefind_), "caps","");

    // g_signal_connect (G_OBJECT (udpsink_), "client-added",  
    // 		      (GCallback)  on_client_added, (gpointer) this);
    // g_signal_connect (G_OBJECT (udpsink_), "client-removed",  
    // 		      (GCallback)  on_client_removed, (gpointer) this);
    
     //registering add_client
    register_method("add_client",
     		    (void *)&add_client_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_INT, NULL),
     		    (gpointer)this);
    set_method_description ("add_client", 
			    "add a client with destination host and port to the list of clients", 
			    //add_client_arg_desc))
			    Method::make_arg_description ("host", 
							  "the hostname/IP address of the client to add",
							  "port",
							  "the port of the client to add",
							  NULL));
    
     //registering remove_client
     register_method("remove_client",
		     (void *)&remove_client_wrapped, 
		     Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_INT, NULL),
     		    (gpointer)this);
      set_method_description ("remove_client", 
			      "remove a client with destination host and port to the list of clients", 
			      Method::make_arg_description ("host",
							    "the hostname/IP address of the client to remove",
							    "port",
							    "the port of the client to remove",
							    NULL));
      

     //registering clear
      register_method("clear",
      		     (void *)&clear_wrapped, 
      		     Method::make_arg_type_description (G_TYPE_NONE),
      		     (gpointer)this);
      set_method_description ("clear", 
			      "remove a client with destination host and port to the list of clients", 
			      Method::make_arg_description ("none",NULL));
     
      
      //registering sink element
      set_sink_element (udpsink_bin_);
      set_on_first_data_hook (UDPSink::add_elements_to_bin,this);
  }
  
  void
  UDPSink::add_elements_to_bin (ShmdataReader *caller, void *udpbin_instance)
  {
    
    UDPSink *context= static_cast<UDPSink *>(udpbin_instance);
    
    caller->set_sink_element (context->udpsink_bin_);
    gst_bin_add (GST_BIN (context->bin_), context->udpsink_bin_);
    gst_element_sync_state_with_parent (context->bin_);
    gst_element_sync_state_with_parent (context->udpsink_bin_);

    gst_bin_add_many (GST_BIN (context->udpsink_bin_),
		      context->typefind_,
		      context->udpsink_,
		      NULL);
    gst_element_link (context->typefind_,
     		      context->udpsink_);
    gst_element_sync_state_with_parent (context->typefind_);
    gst_element_sync_state_with_parent (context->udpsink_);
    
    GstPad *sink_pad = gst_element_get_static_pad (context->typefind_, "sink");
    GstPad *ghost_sinkpad = gst_ghost_pad_new (NULL, sink_pad);
    gst_pad_set_active(ghost_sinkpad,TRUE);
    gst_element_add_pad (context->udpsink_bin_, ghost_sinkpad); 
    gst_object_unref (sink_pad);
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
