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

#include "switcher/rtpsession.h"

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

  void 
  RtpSession::make_rtpsession ()
  {
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
    std::vector<GType> add_data_stream_arg_types;
    add_data_stream_arg_types.push_back (G_TYPE_STRING);
    register_method("add_data_stream",
		    (void *)&add_data_stream_wrapped, 
		    add_data_stream_arg_types,
		    (gpointer)this);
    std::vector<std::pair<std::string,std::string> > arg_desc;
    std::pair<std::string,std::string> socket;
    socket.first = "socket";
    socket.second = "socket path of the shmdata to add to the session (sending)";
    arg_desc.push_back (socket); 
    if (!set_method_description ("add_data_stream", 
				 "add a data stream to the RTP session (sending)", 
				 arg_desc))
      g_printerr ("RTP session: cannot set method description for \"add_data_stream\"\n");

    //registering remove_data_stream
    std::vector<GType> remove_data_stream_arg_types;
    remove_data_stream_arg_types.push_back (G_TYPE_STRING);
    register_method("remove_data_stream",
		    (void *)&remove_data_stream_wrapped, 
		    remove_data_stream_arg_types,
		    (gpointer)this);
    std::vector<std::pair<std::string,std::string> > arg_desc_remove;
    std::pair<std::string,std::string> socket_to_remove;
    socket_to_remove.first = "socket";
    socket_to_remove.second = "socket path of the shmdata to remove from the session (sending)";
    arg_desc_remove.push_back (socket_to_remove); 
    if (!set_method_description ("remove_data_stream", 
				 "remove a data stream from the RTP session (sending)", 
				 arg_desc_remove))
      g_printerr ("RTP session: cannot set method description for \"remove_data_stream\"\n");



    //set the name before registering properties
    name_ = gst_element_get_name (rtpsession_);
    
    make_sdp_init ();
  }

  void
  RtpSession::make_sdp_init ()
  {
    // //prepare SDP description
    // gst_sdp_message_new (&sdp_description_);
    
    // /* some standard things first */
    // gst_sdp_message_set_version (sdp_description_, "0");
    
    // //FIXME check and chose between IP4 and IP6, IP4 hardcoded
    // gst_sdp_message_set_origin (sdp_description_, 
    // 				"-",                // the user name
    // 				"1188340656180883", // a session id
    // 				"1",                // a session version
    // 				"IN",               // a network type
    // 				"IP4",              // an address type
    // 				"localhost"); //an address

    // gst_sdp_message_set_session_name (sdp_description_, "switcher session");
    // gst_sdp_message_set_information (sdp_description_, "telepresence");
    // gst_sdp_message_add_time (sdp_description_, "0", "0", NULL);
    // gst_sdp_message_add_attribute (sdp_description_, "tool", "switcher (with GStreamer)");
    // gst_sdp_message_add_attribute (sdp_description_, "type", "broadcast");
    // gst_sdp_message_add_attribute (sdp_description_, "control", "*");

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
    
    /* now link all to the rtpbin, start by getting an RTP sinkpad for session 0 */
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
    gchar *session_id = g_strdup(rtp_session_array[3]);
    context->local_stream_rtp_id_.insert (reader->get_path (), session_id);
    g_free (rtp_sink_pad_name);
    g_strfreev (rtp_session_array);

    // rtp src pad (is a static pad since created after the request of rtp sink pad)
    gchar *rtp_src_pad_name = g_strconcat ("send_rtp_src_", session_id, NULL); 
    g_print ("RtpSession: request rtp src pad and create a corresponding shmwriter %s\n",rtp_src_pad_name);
    GstPad *rtp_src_pad = gst_element_get_static_pad (context->rtpsession_, rtp_src_pad_name);
    if (!GST_IS_PAD (rtp_src_pad)) 
      g_printerr ("RtpSession: rtp_src_pad is not a pad\n"); 
    ShmdataWriter::ptr rtp_writer;
    rtp_writer.reset (new ShmdataWriter ());
    std::string rtp_writer_name = context->make_shmdata_writer_name (rtp_src_pad_name); 
    rtp_writer->set_path (rtp_writer_name.c_str ());
    rtp_writer->plug (context->bin_, rtp_src_pad);
    context->shmdata_writers_.insert (rtp_writer_name, rtp_writer);
    g_free (rtp_src_pad_name);

    //rtcp src pad
    gchar *rtcp_src_pad_name = g_strconcat ("send_rtcp_src_", session_id,NULL); 
    g_print ("RtpSession: request rtcp src pad and create shmwriter %s\n",rtcp_src_pad_name);
    GstPad *rtcp_src_pad = gst_element_get_request_pad (context->rtpsession_, rtcp_src_pad_name);
    ShmdataWriter::ptr rtcp_writer;
    rtcp_writer.reset (new ShmdataWriter ());
    std::string rtcp_writer_name = context->make_shmdata_writer_name (rtcp_src_pad_name); 
    rtcp_writer->set_path (rtcp_writer_name.c_str());
    rtcp_writer->plug (context->bin_, rtcp_src_pad);
    context->shmdata_writers_.insert (rtcp_writer_name, rtcp_writer);
    g_free (rtcp_src_pad_name);
    
    // We also want to receive RTCP, request an RTCP sinkpad for given session and
    // link it to a funnel for future linking with network connections
    GstElement *funnel = gst_element_factory_make ("funnel", NULL);
    gst_bin_add (GST_BIN (context->bin_), funnel);
    gst_element_sync_state_with_parent (funnel);
    GstPad *funnel_src_pad = gst_element_get_static_pad (funnel, "src");
    gchar *rtcp_sink_pad_name = g_strconcat ("recv_rtcp_sink_", session_id,NULL); 
    GstPad *rtcp_sink_pad = gst_element_get_request_pad (context->rtpsession_, rtcp_sink_pad_name);
    if (gst_pad_link (funnel_src_pad, rtcp_sink_pad) != GST_PAD_LINK_OK)
      g_error ("RtpSession::make_data_stream_available: Failed to link rtcpsrc to rtpbin");
    gst_object_unref (funnel_src_pad);
    g_free (rtcp_sink_pad_name);
    //for cleaning of funnel
    ShmdataHelper::ptr funnel_cleaning;
    funnel_cleaning.reset (new ShmdataHelper ());
    funnel_cleaning->add_element_to_remove (funnel);
    context->rtp_id_funnel_.insert (session_id,funnel_cleaning);
    g_free (session_id);

    // /* the udp sinks and source we will use for RTP and RTCP */
    // rtpsink = gst_element_factory_make ("udpsink", NULL);
    // g_object_set (rtpsink, "port", 10000, "host", "localhost", NULL);

    // rtcpsink = gst_element_factory_make ("udpsink", NULL);
    // g_object_set (rtcpsink, "port", 10001, "host", "localhost", NULL);
    // /* no need for synchronisation or preroll on the RTCP sink */
    // g_object_set (rtcpsink, "async", FALSE, "sync", FALSE, NULL);

    //rtcpsrc = gst_element_factory_make ("udpsrc", NULL);
    // g_object_set (rtcpsrc, "port", 10005, NULL);

     //gst_bin_add_many (GST_BIN (context->bin_), /*rtpsink, rtcpsink,*/ rtcpsrc, NULL);


    //gst_element_link (pay, context->rtpsession_);
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
    if (runtime_ != NULL) // starting the reader if runtime is set
      reader->start ();
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
    if (!local_stream_rtp_id_.contains (shmdata_socket_path))
      {
	g_printerr ("RtpSession::remove_data_stream: %s not present",shmdata_socket_path.c_str ());
	return false;
      }
    std::string id = local_stream_rtp_id_.lookup (shmdata_socket_path);
    
    shmdata_writers_.remove (make_shmdata_writer_name ("send_rtp_src_"+id));
    shmdata_writers_.remove (make_shmdata_writer_name ("send_rtcp_src_"+id));
    shmdata_readers_.remove (shmdata_socket_path);
    shmdata_readers_.remove (make_shmdata_writer_name ("recv_rtcp_sink_"+id));
    local_stream_rtp_id_.remove (shmdata_socket_path);
    
    if (!rtp_id_funnel_.contains (id))
      {
	g_printerr ("RtpSession::remove_data_stream: no funnel");
	return false;
      }
    rtp_id_funnel_.remove (id);

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
