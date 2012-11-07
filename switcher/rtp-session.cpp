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

#include "switcher/rtp-session.h"
#include <sstream>
#include <gst/sdp/gstsdpmessage.h>
#include <glib/gstdio.h> //writing sdp file

namespace switcher
{
  
  QuiddityDocumentation RtpSession::doc_ ("RTP session", "rtpsession",
					  "RTP session manager");
  
  RtpSession::RtpSession ()
  {
    make_rtpsession ();
  }

  RtpSession::RtpSession (QuiddityLifeManager::ptr life_manager)
  {
    life_manager_ = life_manager;
    make_rtpsession ();
  }

  RtpSession::~RtpSession ()
  {
    g_print ("rtpsession deleting\n");

    //removing sdp files
    std::vector <std::string> destinations = destinations_.get_keys ();
    std::vector <std::string>::iterator iter;
    for (iter = destinations.begin (); iter != destinations.end (); iter++)
      {
	std::string sdp_file = make_file_name (*iter);
	sdp_file.append(".sdp");
	g_remove (sdp_file.c_str ());
      }

    //removing rtpsession
    gst_element_set_state (rtpsession_, GST_STATE_NULL);
    GstObject *parent = gst_element_get_parent (rtpsession_);
    if (GST_IS_BIN (parent))
      gst_bin_remove (GST_BIN (parent), rtpsession_);

    g_print ("rtpsession deleted\n");

  }

  void 
  RtpSession::make_rtpsession ()
  {
    next_id_ = 79; //this value is arbitrary and can be changed

    rtpsession_ = gst_element_factory_make ("gstrtpbin",NULL);

    g_signal_connect (G_OBJECT (rtpsession_), "on-bye-ssrc", 
		      (GCallback) on_bye_ssrc, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_bye_timeout", 
		      (GCallback) on_bye_timeout, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_new_ssrc", 
		      (GCallback) on_new_ssrc, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_npt_stop", 
		      (GCallback)  on_npt_stop, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_sender_timeout",  
		      (GCallback) on_sender_timeout, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_ssrc_active",  
		      (GCallback)  on_ssrc_active, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_ssrc_collision",  
		      (GCallback) on_ssrc_collision, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_ssrc_sdes",  
		      (GCallback)  on_ssrc_sdes, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_ssrc_validated",  
		      (GCallback) on_ssrc_validated, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_timeout",  
		      (GCallback) on_timeout, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "pad-added",  
		      (GCallback) on_pad_added, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "pad-removed",  
		      (GCallback) on_pad_removed, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "no-more-pads",  
		      (GCallback) on_no_more_pad, (gpointer) this);

    gst_bin_add (GST_BIN (bin_), rtpsession_);


    //registering add_data_stream
    register_method("add_data_stream",
		    (void *)&add_data_stream_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ("add_data_stream", 
			    "add a data stream to the RTP session (sending)", 
			    Method::make_arg_description ("socket", 
							  "shmdata socket path to add to the session",
							  NULL));

    //registering remove_data_stream
    register_method("remove_data_stream", 
		    (void *)&remove_data_stream_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ("remove_data_stream", 
			    "remove a data stream from the RTP session (sending)", 
			    Method::make_arg_description ("socket", 
							  "shmdata socket path to remove from the session", 
							  NULL));
    
    //registering add_dest
    register_method("add_destination", 
		    (void *)&add_destination_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ("add_destination", 
			    "add a destination (two destinations can share the same host name)", 
			    Method::make_arg_description ("nick_name", 
							  "a destination name (user defined)",
							  "host_name",
							  "the host name of the destination",
							  NULL));

    //registering remove_dest
    register_method("remove_destination", 
		    (void *)&remove_destination_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ("remove_destination", 
			    "remove a destination", 
			    Method::make_arg_description ("nick_name", 
							  "the destination name",
							  NULL));

    //registering "add_udp_stream_to_dest"
    register_method("add_udp_stream_to_dest",
		    (void *)&add_udp_stream_to_dest_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING,G_TYPE_STRING,G_TYPE_STRING,NULL),
		    (gpointer)this);
    set_method_description ("add_udp_stream_to_dest", 
			    "stream RTP to a port with udp", 
			    Method::make_arg_description ("socket", "local socket path of the shmdata",
							  "dest", "name of the destination",
							  "port", "destination port",
							  NULL));

    //registering "remove_udp_dest"
    register_method("remove_udp_dest",
		    (void *)&remove_udp_dest_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING,G_TYPE_STRING,G_TYPE_STRING,NULL),
		    (gpointer)this);
    set_method_description ("remove_udp_dest", 
			    "remove destination", 
			    Method::make_arg_description ("socket", "local socket path of the shmdata",
							  "host", "destination host",
							  "port", "destination port",
							  NULL));

    //registering "write_sdp_file"
    register_method("write_sdp_file",
		    (void *)&write_sdp_file_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING,NULL),
		    (gpointer)this);
    set_method_description ("write_sdp_file", 
			    "print sdp for the given destination", 
			    Method::make_arg_description ("nickname", 
							  "the name of the destination",
							  NULL));
   

    //set the name before registering properties
    set_name (gst_element_get_name (rtpsession_));
    
  }
  
  gboolean
  RtpSession::write_sdp_file_wrapped (gpointer nick_name, 
				 gpointer user_data)
  {
    RtpSession *context = static_cast<RtpSession*>(user_data);
    
    if (context->write_sdp_file ((char *)nick_name))
      return TRUE;
    else
      return FALSE;
  }


  bool
  RtpSession::write_sdp_file (std::string dest_name)
  {
    if (!destinations_.contains (dest_name))
      {
	g_printerr ("RtpSession: destination named %s does not exists, cannot print sdp",
		    dest_name.c_str ());
	return false;
      }
    std::string res = destinations_.lookup (dest_name)->get_sdp();
    
    std::string sdp_file = make_file_name (dest_name);
    sdp_file.append(".sdp");

    g_remove (sdp_file.c_str ());
    
    if (!g_file_set_contents (sdp_file.c_str (), 
			      res.c_str (), 
			      -1, //no size, res is a null terminated string
			      NULL)) //not getting errors
      return false;
    
    return true;
  }


  QuiddityDocumentation 
  RtpSession::get_documentation ()
  {
    return doc_;
  }

  //function used as a filter for selecting the appropriate rtp payloader
  gboolean
  RtpSession::sink_factory_filter (GstPluginFeature * feature, gpointer data)
  {
    // guint rank;
    const gchar *klass;
    
    GstCaps* caps = (GstCaps *)data;

    // searching element factories only
    if (!GST_IS_ELEMENT_FACTORY (feature))
      return FALSE;
    
    klass = gst_element_factory_get_klass (GST_ELEMENT_FACTORY (feature));
    if (!(g_strrstr (klass, "Payloader") && g_strrstr (klass, "RTP")))
      return FALSE;
    
    if (!gst_element_factory_can_sink_caps (GST_ELEMENT_FACTORY (feature), caps))
      return FALSE;
    
    return TRUE;
  }
  
  //sorting factory by rank
  gint
  RtpSession::sink_compare_ranks (GstPluginFeature * f1, GstPluginFeature * f2)
  {
    gint diff;
    
    diff = gst_plugin_feature_get_rank (f2) - gst_plugin_feature_get_rank (f1);
    if (diff != 0)
      return diff;
    return g_strcmp0 (gst_plugin_feature_get_name (f2),
		      gst_plugin_feature_get_name (f1));
  }

  //this is a typefind function, called when type of input stream from a shmdata is found
  void
  RtpSession::make_data_stream_available (GstElement* typefind, 
					  guint probability, 
					  GstCaps *caps, 
					  gpointer user_data)
  {
    RtpSession *context = static_cast<RtpSession *>(user_data);
    
    GstElement *pay;
    GList *list = gst_registry_feature_filter (gst_registry_get_default (),
					       (GstPluginFeatureFilter) sink_factory_filter, 
					       FALSE, caps);
    list = g_list_sort (list, (GCompareFunc) sink_compare_ranks);

    if (list != NULL)
      pay = gst_element_factory_create (GST_ELEMENT_FACTORY (list->data), NULL);
    else
      pay = gst_element_factory_make ("rtpgstpay", NULL);

    ShmdataReader *reader= (ShmdataReader *) g_object_get_data (G_OBJECT (typefind),"shmdata-reader");
    reader->add_element_to_remove (pay);
        
    g_print ("RtpSession::make_data_stream_available: %s payloader\n",GST_ELEMENT_NAME (pay));

    /* add capture and payloading to the pipeline and link */
    gst_bin_add_many (GST_BIN (context->bin_), pay, NULL);
    gst_element_link (typefind, pay);
    gst_element_sync_state_with_parent (pay);
    
    /* now link all to the rtpbin, start by getting an RTP sinkpad for session "%d" */
    GstPad *srcpad, *sinkpad;
    sinkpad = gst_element_get_request_pad (context->rtpsession_, "send_rtp_sink_%d");
    srcpad = gst_element_get_static_pad (pay, "src");
    if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK)
      g_error ("RtpSession::make_data_stream_available: failed to link payloader to rtpbin");
    gst_object_unref (sinkpad);
    gst_object_unref (srcpad);
    
    //get name for the newly created pad
    gchar *rtp_sink_pad_name = gst_pad_get_name (sinkpad);
    gchar **rtp_session_array = g_strsplit_set (rtp_sink_pad_name, "_",0);
    
    gchar *rtp_session_id = g_strdup(rtp_session_array[3]);
    context->rtp_ids_.insert (reader->get_path (), rtp_session_id);
    //using internal session id for this local stream
    std::string internal_session_id (context->internal_id_.lookup (reader->get_path ()));
    
    g_free (rtp_sink_pad_name);
    g_strfreev (rtp_session_array);

    // rtp src pad (is a static pad since created after the request of rtp sink pad)
    gchar *rtp_src_pad_name = g_strconcat ("send_rtp_src_", rtp_session_id, NULL); 
    g_print ("RtpSession: request rtp src pad and create a corresponding shmwriter %s\n",rtp_src_pad_name);
    GstPad *rtp_src_pad = gst_element_get_static_pad (context->rtpsession_, rtp_src_pad_name);
    if (!GST_IS_PAD (rtp_src_pad)) 
      g_printerr ("RtpSession: rtp_src_pad is not a pad\n"); 
    ShmdataWriter::ptr rtp_writer;
    rtp_writer.reset (new ShmdataWriter ());
    std::string rtp_writer_name = context->make_file_name ("send_rtp_src_"+internal_session_id); 
    rtp_writer->set_path (rtp_writer_name.c_str ());
    rtp_writer->plug (context->bin_, rtp_src_pad);
    context->internal_shmdata_writers_.insert (rtp_writer_name, rtp_writer);
    g_free (rtp_src_pad_name);

    //rtcp src pad
    gchar *rtcp_src_pad_name = g_strconcat ("send_rtcp_src_", rtp_session_id,NULL); 
    g_print ("RtpSession: request rtcp src pad %s\n",rtcp_src_pad_name);
    GstPad *rtcp_src_pad = gst_element_get_request_pad (context->rtpsession_, rtcp_src_pad_name);
    ShmdataWriter::ptr rtcp_writer;
    rtcp_writer.reset (new ShmdataWriter ());
    std::string rtcp_writer_name = context->make_file_name ("send_rtcp_src_"+internal_session_id); 
    rtcp_writer->set_path (rtcp_writer_name.c_str());
    rtcp_writer->plug (context->bin_, rtcp_src_pad);
    context->internal_shmdata_writers_.insert (rtcp_writer_name, rtcp_writer);
    g_free (rtcp_src_pad_name);
    
    // We also want to receive RTCP, request an RTCP sinkpad for given session and
    // link it to a funnel for future linking with network connections
    GstElement *funnel = gst_element_factory_make ("funnel", NULL);
    gst_bin_add (GST_BIN (context->bin_), funnel);
    gst_element_sync_state_with_parent (funnel);
    GstPad *funnel_src_pad = gst_element_get_static_pad (funnel, "src");
    gchar *rtcp_sink_pad_name = g_strconcat ("recv_rtcp_sink_", rtp_session_id,NULL); 
    GstPad *rtcp_sink_pad = gst_element_get_request_pad (context->rtpsession_, rtcp_sink_pad_name);
    if (gst_pad_link (funnel_src_pad, rtcp_sink_pad) != GST_PAD_LINK_OK)
      g_error ("RtpSession::make_data_stream_available: Failed to link rtcpsrc to rtpbin");
    gst_object_unref (funnel_src_pad);
    g_free (rtcp_sink_pad_name);
    //for cleaning of funnel
    GstElementCleaner::ptr funnel_cleaning;
    funnel_cleaning.reset (new GstElementCleaner ());
    funnel_cleaning->add_element_to_remove (funnel);
    context->funnels_.insert (reader->get_path (),funnel_cleaning);
    g_free (rtp_session_id);
  }

  void 
  RtpSession::attach_data_stream(ShmdataReader *caller, void *rtpsession_instance)
  {
    RtpSession *context = static_cast<RtpSession *>(rtpsession_instance);
    GstElement *funnel, *typefind;
    funnel = gst_element_factory_make ("funnel", NULL);
    typefind = gst_element_factory_make ("typefind", NULL);
    //give caller to typefind in order to register telement to remove
    g_object_set_data (G_OBJECT (typefind), "shmdata-reader", (gpointer)caller);
    g_signal_connect (typefind, "have-type", G_CALLBACK (RtpSession::make_data_stream_available), context);
    gst_bin_add_many (GST_BIN (context->bin_), funnel, typefind, NULL);
    gst_element_link (funnel, typefind);
    gst_element_sync_state_with_parent (funnel);
    gst_element_sync_state_with_parent (typefind);
    caller->set_sink_element (funnel);
    caller->add_element_to_remove (funnel);
    caller->add_element_to_remove (typefind);
  }
  


  gboolean
  RtpSession::add_destination_wrapped (gpointer nick_name, 
				       gpointer host_name,
				       gpointer user_data)
  {
    RtpSession *context = static_cast<RtpSession*>(user_data);
       
    if (context->add_destination ((char *)nick_name,(char *)host_name))
      return TRUE;
    else
      return FALSE;
  }
  
  bool 
  RtpSession::add_destination (std::string nick_name,std::string host_name)
  {
    if (destinations_.contains (nick_name))
      {
	g_printerr ("RtpSession: a destination named %s already exists, cannot add",
		    nick_name.c_str ());
	return false;
      }
    RtpDestination::ptr dest;
    dest.reset (new RtpDestination ());
    dest->set_host_name (host_name);
    destinations_.insert (nick_name, dest);
    return true;
  }

  gboolean
  RtpSession::remove_destination_wrapped (gpointer nick_name, 
					  gpointer user_data)
  {
    RtpSession *context = static_cast<RtpSession*>(user_data);
       
    if (context->remove_destination ((char *)nick_name))
      return TRUE;
    else
      return FALSE;
  }
  
  bool 
  RtpSession::remove_destination (std::string nick_name)
  {
    if (!destinations_.contains (nick_name))
      {
	g_printerr ("RtpSession: destination named %s does not exists, cannot remove",
		    nick_name.c_str ());
	return false;
      }
    //FIXME free streams

    
    destinations_.remove (nick_name);
    return true;
  }




  gboolean
  RtpSession::add_udp_stream_to_dest_wrapped (gpointer shmdata_name, 
					      gpointer nick_name, 
					      gpointer port, 
					      gpointer user_data)
  {
    RtpSession *context = static_cast<RtpSession*>(user_data);
       
    if (context->add_udp_stream_to_dest ((char *)shmdata_name,(char *)nick_name,(char *)port))
      return TRUE;
    else
      return FALSE;
  }
  
 
  bool
  RtpSession::add_udp_stream_to_dest (std::string shmdata_socket_path, std::string nick_name, std::string port)
  {
    if (!internal_id_.contains (shmdata_socket_path))
      {
	g_printerr ("RtpSession is not connected to %s\n",shmdata_socket_path.c_str ());
	return false;
      }
    
    if (!destinations_.contains (nick_name))
      {
	g_printerr ("RtpSession does not contain a destination named %s\n",nick_name.c_str ());
	return false;
      }

    gint rtp_port = atoi(port.c_str());

    if (rtp_port %2 !=0)
      {
	g_printerr ("RtpSession rtp destination port %s must be even, not odd\n",port.c_str ());
	return false;
      }
    std::string id = internal_id_.lookup (shmdata_socket_path);

    if (!quiddity_managers_.contains (shmdata_socket_path))
      {
	//creating an internal quiddity manager 
	QuiddityManager::ptr manager = QuiddityManager::make_manager ("manager_"+get_name()+"_"+id);
	quiddity_managers_.insert (shmdata_socket_path, manager);
	manager->create ("runtime","single_runtime");//only one runtime for all

	std::vector <std::string> arg;
	manager->create ("udpsink","udpsend_rtp");
	manager->create ("udpsink","udpsend_rtcp");
	arg.push_back ("single_runtime");
	manager->invoke ("udpsend_rtp","set_runtime",arg);
	manager->invoke ("udpsend_rtcp","set_runtime",arg);
	
	arg.clear ();
	arg.push_back (make_file_name ("send_rtp_src_"+id));
	manager->invoke ("udpsend_rtp","connect",arg);

	arg.clear ();
	arg.push_back (make_file_name ("send_rtcp_src_"+id));
	manager->invoke ("udpsend_rtcp","connect",arg);
      }

    QuiddityManager::ptr manager = quiddity_managers_.lookup (shmdata_socket_path);
    
    //rtp stream
    RtpDestination::ptr dest = destinations_.lookup (nick_name);
    
    dest->add_stream (shmdata_socket_path,manager,port);
    std::vector <std::string> arg;
    arg.push_back (dest->get_host_name ());
    arg.push_back (port);
    manager->invoke ("udpsend_rtp","add_client",arg);
    
    //rtcp stream
    arg.clear ();
    arg.push_back (dest->get_host_name ());
    std::ostringstream rtcp_port;
    rtcp_port << rtp_port + 1;
    arg.push_back (rtcp_port.str());
    manager->invoke ("udpsend_rtcp","add_client",arg);

    //TODO connect to funnel for rtcp receiving
    return true;
  }

  
  gboolean 
  RtpSession::remove_udp_dest_wrapped (gpointer shmdata_socket_path, 
				       gpointer host, 
				       gpointer port,
				       gpointer user_data)
  {
    RtpSession *context = static_cast<RtpSession*>(user_data);

    if (context->remove_udp_dest ((char *)shmdata_socket_path, (char *)host, (char *)port))
      return TRUE;
    else
      return FALSE;
  }
  
 bool
  RtpSession::remove_udp_dest (std::string shmdata_socket_path, std::string host, std::string port)
  {
    

    if (!internal_id_.contains (shmdata_socket_path))
      {
	g_printerr ("RtpSession is not connected to %s\n",shmdata_socket_path.c_str ());
	return false;
      }
    
    QuiddityManager::ptr manager = quiddity_managers_.lookup (shmdata_socket_path);
    
    //rtp
    std::vector <std::string> arg;
    arg.push_back (host);
    arg.push_back (port);
    manager->invoke ("udpsend_rtp","remove_client",arg);
    
    //rtcp
    arg.clear ();
    gint rtp_port = atoi(port.c_str());
    arg.push_back (host);
    std::ostringstream rtcp_port;
    rtcp_port << rtp_port + 1;
    arg.push_back (rtcp_port.str());
    manager->invoke ("udpsend_rtp","remove_client",arg);
    
    return true;
  }

  gboolean
  RtpSession::add_data_stream_wrapped (gpointer connector_name, gpointer user_data)
  {
    RtpSession *context = static_cast<RtpSession*>(user_data);
       
    if (context->add_data_stream ((char *)connector_name))
      return TRUE;
    else
      return FALSE;
  }

  bool
  RtpSession::add_data_stream (std::string shmdata_socket_path)
  {
    ShmdataReader::ptr reader;
    reader.reset (new ShmdataReader ());
    reader->set_path (shmdata_socket_path.c_str());
    reader->set_bin (bin_);
    reader->set_on_first_data_hook (attach_data_stream, this);
    if (runtime_) // starting the reader if runtime is set
      reader->start ();
    //saving info about this local stream
    std::ostringstream os_id;
    os_id << next_id_;
    next_id_++;
    internal_id_.insert (shmdata_socket_path, os_id.str());
    shmdata_readers_.insert (shmdata_socket_path, reader);
    return true;
  }  

  gboolean
  RtpSession::remove_data_stream_wrapped (gpointer connector_name, gpointer user_data)
  {
    //std::string connector = static_cast<std::string>(connector_name);
    RtpSession *context = static_cast<RtpSession*>(user_data);
       
    if (context->remove_data_stream ((char *)connector_name))
      return TRUE;
    else
      return FALSE;
  }

  bool
  RtpSession::remove_data_stream (std::string shmdata_socket_path)
  {
    if (!internal_id_.contains (shmdata_socket_path))
      {
	g_printerr ("RtpSession::remove_data_stream: %s not present",shmdata_socket_path.c_str ());
	return false;
      }

    

    std::string id = internal_id_.lookup (shmdata_socket_path);
    internal_id_.remove (shmdata_socket_path);
    
    internal_shmdata_writers_.remove (make_file_name ("send_rtp_src_"+id));
    internal_shmdata_writers_.remove (make_file_name ("send_rtcp_src_"+id));
    shmdata_readers_.remove (shmdata_socket_path);
    internal_shmdata_readers_.remove (make_file_name ("recv_rtcp_sink_"+id));
    
    if(quiddity_managers_.contains (shmdata_socket_path))
      {
	quiddity_managers_.remove (shmdata_socket_path);
      }
    rtp_ids_.remove (shmdata_socket_path);


    if (!funnels_.contains (shmdata_socket_path))
      {
	g_printerr ("RtpSession::remove_data_stream: no funnel");
	return false;
      }
    funnels_.remove (shmdata_socket_path);
    
    
    g_print ("data_stream removed\n");
    return true;
  }

  void
  RtpSession::on_bye_ssrc (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_print ("on_bye_ssrc\n");
  }

  void
  RtpSession::on_bye_timeout (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_print ("on_bye_timeout\n");
  }

  void
  RtpSession::on_new_ssrc (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_print ("on_new_ssrc\n");
  }

  void
  RtpSession::on_npt_stop (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_print ("on_npt_stop\n");
  }

  void
  RtpSession::on_sender_timeout (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_print ("on_sender_timeout\n");
  }

  void
  RtpSession::on_ssrc_active  (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_print ("on_ssrc_active\n");
  }

  void
  RtpSession::on_ssrc_collision (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_print ("on_ssrc_active\n");
  }

  void
  RtpSession::on_ssrc_sdes  (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_print ("on_ssrc_sdes\n");
  }

  void
  RtpSession::on_ssrc_validated (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_print ("on_ssrc_validated\n");
  }
 
  void
  RtpSession::on_timeout  (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_print ("on_timeout\n");
  }

  void
  RtpSession::on_pad_added (GstElement *gstelement, GstPad *new_pad, gpointer user_data) 
  {
    g_print ("on_pad_added, name: %s, direction: %d\n", 
	     gst_pad_get_name(new_pad),
	     gst_pad_get_direction (new_pad));
    // RtpSession *context = static_cast<RtpSession *>(user_data);
  }

  void
  RtpSession::on_pad_removed (GstElement *gstelement, GstPad *new_pad, gpointer user_data) 
  {
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_print ("on_pad_removed, name: %s, direction: %d\n", 
	     gst_pad_get_name(new_pad),
	     gst_pad_get_direction (new_pad));
    
  }

  void
  RtpSession::on_no_more_pad   (GstElement *gstelement, gpointer user_data) 
  {
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_print ("on_no_more_pad\n");
  }

}
