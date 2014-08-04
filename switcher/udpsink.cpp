/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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

#include "udpsink.h"
#include "gst-utils.h"
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if HAVE_OSX
#include <sys/socket.h>
#endif

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(UDPSink,
				       "UDP Sender",
				       "udp network", 
				       "send data stream with udp",
				       "LGPL",
				       "udpsink",
				       "Nicolas Bouillot");
  UDPSink::UDPSink () :
    udpsink_ (nullptr),
    udpsink_bin_ (nullptr),
    typefind_ (nullptr),
    ghost_sinkpad_ (nullptr)
  {}

  bool 
  UDPSink::init_gpipe ()
  {
    if ( !GstUtils::make_element ("bin", &udpsink_bin_)
	 || !GstUtils::make_element ("typefind", &typefind_)
	 || !GstUtils::make_element ("multiudpsink", &udpsink_))
      return false;

  
    //g_object_set (G_OBJECT (udpsink_bin_), "async-handling", TRUE, nullptr);
    ghost_sinkpad_ = nullptr;

    //set the name before registering properties
    set_name (gst_element_get_name (udpsink_));
    g_object_set (G_OBJECT (udpsink_), "sync", FALSE, nullptr);

#if HAVE_OSX    
    // turnaround for OSX: create sender socket
    int sock;
    if ((sock = socket (AF_INET, SOCK_DGRAM, 0)) < 0)
      {
	g_warning ("udp sink: cannot create socket");
	return false;
      }
    guint bc_val = 1;
    if (setsockopt (sock, SOL_SOCKET, SO_BROADCAST, &bc_val,
            sizeof (bc_val)) < 0)
      {
	g_warning ("udp sink: cannot set broadcast to socket");
	return false;
      }
    g_object_set (G_OBJECT (udpsink_), "sockfd", sock, nullptr);
#endif


    install_property (G_OBJECT (udpsink_),"blocksize","blocksize", "Blocksize");
    install_property (G_OBJECT (udpsink_),"bytes-served","bytes-served", "Bytes Served");
    install_property (G_OBJECT (udpsink_),"clients","clients", "Clients");
    install_property (G_OBJECT (udpsink_),"ttl","ttl", "TTL");
    install_property (G_OBJECT (udpsink_),"ttl-mc","ttl-mc", "TTL-MC");
    install_property (G_OBJECT (udpsink_),"loop","loop", "Loop");
    install_property (G_OBJECT (typefind_), "caps","caps", "Capabilities");

    // g_signal_connect (G_OBJECT (udpsink_), "client-added",  
    // 		      (GCallback)  on_client_added, (gpointer) this);
    // g_signal_connect (G_OBJECT (udpsink_), "client-removed",  
    // 		      (GCallback)  on_client_removed, (gpointer) this);
    
    install_method ("Add Client",
		    "add_client", 
		    "add a client with destination host and port to the list of clients", 
		    "success or fail",
		    Method::make_arg_description ("Host", 
						  "host", 
						  "the hostname/IP address of the client to add",
						  "Port",
						  "port",
						  "the port of the client to add",
						  nullptr),
		    (Method::method_ptr) &add_client_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_INT, nullptr),
     		    this);
   
    install_method ("Remove Client",
		    "remove_client", 
		    "remove a client with destination host and port to the list of clients", 
		    "success or fail",
		    Method::make_arg_description ("Host",
						  "host",
						  "the hostname/IP address of the client to remove",
						  "Port",
						  "port",
						  "the port of the client to remove",
						  nullptr),
		    (Method::method_ptr) &remove_client_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_INT, nullptr),
     		    this);

     
    install_method ("Clear",
		    "clear", 
		    "remove a client with destination host and port to the list of clients", 
		    "success or fail",
		    Method::make_arg_description ("none",nullptr),
		    (Method::method_ptr) &clear_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_NONE, nullptr), 
		    this);
      
    //registering sink element
    set_on_first_data_hook (UDPSink::add_elements_to_bin,this);
    set_sink_element (udpsink_bin_);
      
    return true;
  }

  UDPSink::~UDPSink ()
  {
    g_debug ("removing udpsink %s (%s)", get_nick_name ().c_str (), GST_ELEMENT_NAME (udpsink_bin_));
    
    // GstBin *parent = GST_BIN( GST_ELEMENT_PARENT (udpsink_bin_));
    // g_debug ("num child in parent bin : %d", 
    // 	     GST_BIN_NUMCHILDREN (parent));
    
    if (ghost_sinkpad_ != nullptr)
      {
	if (gst_pad_is_linked (ghost_sinkpad_))
	  {
	    GstPad *peer_pad = gst_pad_get_peer (ghost_sinkpad_);
	    gst_pad_unlink (peer_pad, ghost_sinkpad_);
	    gst_object_unref (peer_pad);
	  }
	gst_pad_set_active(ghost_sinkpad_,FALSE);
	gst_element_remove_pad (udpsink_bin_, ghost_sinkpad_);
      }

    GstUtils::clean_element (typefind_);
    GstUtils::clean_element (udpsink_);
    GstUtils::clean_element (udpsink_bin_);
  }
  
  void
  UDPSink::add_elements_to_bin (ShmdataReader *caller, void *udpbin_instance)
  {
    
    UDPSink *context= static_cast<UDPSink *>(udpbin_instance);
    
    caller->set_sink_element (context->udpsink_bin_);
    gst_bin_add (GST_BIN (context->bin_), context->udpsink_bin_);
    
    
    gst_bin_add_many (GST_BIN (context->udpsink_bin_),
		      context->typefind_,
		      context->udpsink_,
		      nullptr);
    gst_element_link (context->typefind_,
     		      context->udpsink_);
    
    //GstUtils::wait_state_changed (context->udpsink_bin_);
    GstUtils::sync_state_with_parent (context->udpsink_bin_);
    
    GstPad *sink_pad = gst_element_get_static_pad (context->typefind_, "sink");
    context->ghost_sinkpad_ = gst_ghost_pad_new (nullptr, sink_pad);
    gst_pad_set_active(context->ghost_sinkpad_,TRUE);
    gst_element_add_pad (context->udpsink_bin_, context->ghost_sinkpad_); 
    gst_object_unref (sink_pad);
  }

  void 
  UDPSink::on_client_added (GstElement */*multiudpsink*/, 
			    gchar */*host*/, 
			    gint /*port*/, 
			    gpointer /*user_data*/)
  {
    //UDPSink *context = static_cast<UDPSink *>(user_data);
    g_debug ("UDPSink::on_client_added");
  }

  void 
  UDPSink::on_client_removed (GstElement */*multiudpsink*/, 
			      gchar */*host*/, 
			      gint /*port*/, 
			      gpointer /*user_data*/)
  {
    //UDPSink *context = static_cast<UDPSink *>(user_data);
    g_debug ("UDPSink::on_client_removed");
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
    g_signal_emit_by_name (udpsink_, "remove", host, port, nullptr);
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
    g_signal_emit_by_name (udpsink_, "add", host, port, nullptr);
    return true;
  }

  gboolean
  UDPSink::clear_wrapped (gpointer /*unused*/, gpointer user_data)
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
    g_signal_emit_by_name (udpsink_, "clear", nullptr);
    return true;
  }

}
